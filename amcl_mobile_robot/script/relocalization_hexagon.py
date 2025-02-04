#!/usr/bin/env python3

import rospy
import random
import csv
import time
import tf
import math
import psutil
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState

class AMCLNavigationTest:
    def __init__(self):
        rospy.init_node("amcl_navigation_test", anonymous=True)
        
        # Publisher for Pose Estimate
        self.pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)

        # Move Base Client
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()

        # Gazebo Services for Resetting Robot Position
        rospy.wait_for_service("/gazebo/set_model_state")
        rospy.wait_for_service("/gazebo/get_model_state")
        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

        self.data = []
        self.cpu_ram_data = []
        self.total_tests = 0  

        # Initializing AMCL Pose Variables
        self.amcl_pose = None  
        self.covariance_x = None  
        self.covariance_y = None  
        self.robot_name = self.get_robot_name()

        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback)
        rospy.loginfo("✅ Subscribed to /amcl_pose topic...")

    def get_robot_name(self):
        """ Dynamically get the robot model name from Gazebo """
        models = ["/", "mobile_robot", "robot", "turtlebot3", "jackal"]  
        for model in models:
            response = self.get_model_state(model, "world")
            if response.success:
                rospy.loginfo(f"✅ Detected Robot Model in Gazebo: {model}")
                return model
        rospy.logwarn("⚠️ No valid robot model found in Gazebo! Check model name.")
        return None

    def amcl_pose_callback(self, msg):
        """ Update AMCL Pose Data """
        self.amcl_pose = msg.pose.pose.position
        self.covariance_x = msg.pose.covariance[0]
        self.covariance_y = msg.pose.covariance[7]

    def get_current_position(self):
        """ Get Robot's Current Position from AMCL """
        if self.amcl_pose is not None:
            return self.amcl_pose.x, self.amcl_pose.y
        rospy.logwarn("⚠️ AMCL Pose not received yet! Using default (0,0)")
        return 0.0, 0.0  

    def publish_initial_pose(self, x, y, yaw=0.0):
        """ Publish Fake Estimated Pose to AMCL & RViz """
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.orientation.x = quaternion[0]
        pose_msg.pose.pose.orientation.y = quaternion[1]
        pose_msg.pose.pose.orientation.z = quaternion[2]
        pose_msg.pose.pose.orientation.w = quaternion[3]

        self.pose_pub.publish(pose_msg)

    def send_goal(self, x, y):
        """ Send Goal to Move Base """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0

        self.client.send_goal(goal)
        self.client.wait_for_result()

    def reset_position(self):
        """ Reset Robot Position in Gazebo & RViz """
        if not self.robot_name:
            rospy.logerr("❌ No valid robot name found. Cannot reset!")
            return

        # Reset in Gazebo
        state_msg = ModelState()
        state_msg.model_name = self.robot_name
        state_msg.pose.position.x = 0.0
        state_msg.pose.position.y = 0.0
        state_msg.pose.orientation.w = 1.0  

        try:
            self.set_model_state(state_msg)
        except rospy.ServiceException as e:
            rospy.logerr(f"❌ Failed to reset robot position in Gazebo: {e}")

        rospy.sleep(5)
        self.publish_initial_pose(0.0, 0.0, 0.0)

    def record_cpu_ram_usage(self):
        """ Record current CPU and RAM usage """
        timestamp = time.time()
        cpu_usage = psutil.cpu_percent(interval=1)
        ram_usage = psutil.virtual_memory().percent
        self.cpu_ram_data.append([self.total_tests, timestamp, cpu_usage, ram_usage])
        return cpu_usage, ram_usage  # Return these values so they can be logged

    def test_navigation(self, max_tests=10):
        while self.total_tests < max_tests:
            self.total_tests += 1
            rospy.loginfo(f"🔹 Running Test {self.total_tests}/{max_tests}")

            x_true, y_true = self.get_current_position()

            x_pose = x_true + random.uniform(-1.0, 1.0)
            y_pose = y_true + random.uniform(-1.0, 1.0)
            self.publish_initial_pose(x_pose, y_pose)
            rospy.sleep(2)

            x_goal = x_true + random.uniform(-1.0, 1.0)
            y_goal = y_true + random.uniform(-1.0, 1.0)

            start_time = time.time()
            self.send_goal(x_goal, y_goal)
            relocalization_time = (time.time() - start_time) * 1000  

            delta_x = abs(x_true - x_pose)
            delta_y = abs(y_true - y_pose)

            # 🟢 Get CPU and RAM Usage
            cpu_usage, ram_usage = self.record_cpu_ram_usage()

            # 🔥 Condition for Success or Failure
            covariancex_threshold = 0.1  
            covariancey_threshold = 0.01
            relocalization_time_threshold = 40000  # 15 seconds threshold
            if relocalization_time < relocalization_time_threshold and self.covariance_x < covariancex_threshold and self.covariance_y < covariancey_threshold:
                test_status = "Success"
            else:
                test_status = "Failure"

            # 📢 Print Test Results
            rospy.loginfo(f"📝 Test {self.total_tests}: {test_status}")
            rospy.loginfo(f"⏱️ Relocalization Time: {relocalization_time:.2f} ms")
            rospy.loginfo(f"🎯 Goal Position: X={x_goal:.2f}, Y={y_goal:.2f}")
            rospy.loginfo(f"📏 Fake Pose Error: ΔX={delta_x:.2f}, ΔY={delta_y:.2f}")
            rospy.loginfo(f"📊 Covariance X: {self.covariance_x:.4f}, Covariance Y: {self.covariance_y:.4f}")
            rospy.loginfo(f"🔥 CPU Usage: {cpu_usage:.2f}%, RAM Usage: {ram_usage:.2f}%")

            # 📌 Save Test Data
            self.data.append([
                self.total_tests, x_true, y_true, x_pose, y_pose, x_goal, y_goal,
                delta_x, delta_y, self.covariance_x, self.covariance_y, 
                relocalization_time, cpu_usage, ram_usage, test_status
            ])

            self.reset_position()
            time.sleep(1)

        self.save_data()
        self.save_cpu_ram_data()

    def save_data(self):
        """ Save Test Results to CSV """
        filename = "amcl_navigation_results_unknow.csv"
        with open(filename, "w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["Test #", "True X", "True Y", "Fake Pose X", "Fake Pose Y", 
                            "Goal X", "Goal Y", "ΔX (Error)", "ΔY (Error)", 
                            "Covariance X", "Covariance Y", "Relocalization Time (ms)", 
                            "CPU Usage (%)", "RAM Usage (%)", "Status"])
            for row in self.data:
                writer.writerow(row)
        rospy.loginfo(f"✅ Data saved to {filename}")

    def save_cpu_ram_data(self):
        """ Save CPU & RAM Usage Data to CSV """
        filename = "cpu_ram_usage_unknow.csv"
        with open(filename, "w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["Test #", "Timestamp", "CPU Usage (%)", "RAM Usage (%)"])
            for row in self.cpu_ram_data:
                writer.writerow(row)
        rospy.loginfo(f"✅ CPU & RAM Data saved to {filename}")



if __name__ == "__main__":
    tester = AMCLNavigationTest()
    tester.test_navigation(max_tests=10)
