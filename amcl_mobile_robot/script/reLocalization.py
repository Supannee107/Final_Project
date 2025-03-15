#!/usr/bin/env python3


import os
import rospy
import random
import csv
import time
import threading
from datetime import datetime
import psutil
import tf
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import PoseWithCovarianceStamped
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class AMCLNavigationTest:
    def __init__(self):
        rospy.init_node("amcl_navigation_test", anonymous=True)

        self.pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        rospy.wait_for_service("/gazebo/set_model_state")
        rospy.wait_for_service("/gazebo/get_model_state")
        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

        self.amcl_pose = None
        self.covariance_x = None
        self.covariance_y = None
        self.total_tests = 0
        self.data = []

        self.robot_name = self.get_robot_name()

        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback)

        # ✅ Ensure directories exist at the start
        self.result_directory = "/home/supannee/Project/src/Final_Project/amcl_mobile_robot/result/Normal_Ob/ReLocalization/iter1/"
        os.makedirs(self.result_directory, exist_ok=True)

    def get_robot_name(self):
        models = ["mobile_robot", "robot", "/"]
        for model in models:
            resp = self.get_model_state(model, "")
            if resp.success:
                return model
        rospy.logerr("❌ No robot model found! Check Gazebo model names.")
        rospy.signal_shutdown("No valid robot model found.")
        return None

    def amcl_pose_callback(self, msg):
        self.amcl_pose = msg.pose.pose.position
        self.covariance_x = msg.pose.covariance[0]
        self.covariance_y = msg.pose.covariance[7]

    def publish_initial_pose(self, x, y, yaw=0.0):
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

    def record_cpu_ram_continuous(self):
        """ Continuously records CPU & RAM usage into a CSV file. """
        filepath = os.path.join(self.result_directory, "system_usage.csv")
        
        with open(filepath, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Timestamp', 'CPU Usage (%)', 'RAM Usage (%)'])
            while self.monitoring_active:
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                cpu_usage = psutil.cpu_percent()
                ram_usage = psutil.virtual_memory().percent
                writer.writerow([timestamp, cpu_usage, ram_usage])
                time.sleep(0.1)

    def send_goal(self, x, y):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(goal)
        self.client.wait_for_result()

    def reset_position(self):
        state = ModelState()
        state.model_name = self.robot_name
        state.pose.position.x = 0
        state.pose.position.y = 0
        state.pose.orientation.w = 1.0
        self.set_model_state(state)
        rospy.sleep(2)

    def test_navigation(self, max_tests=10):
        self.monitoring_active = True
        threading.Thread(target=self.record_cpu_ram_continuous).start()

        filepath = os.path.join(self.result_directory, "navigation_test_results.csv")
        with open(filepath, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Test #', 'True X', 'True Y', 'Fake Pose X', 'Fake Pose Y',
                             'Goal X', 'Goal Y', 'ΔX (Error)', 'ΔY (Error)', 'Covariance X',
                             'Covariance Y', 'Relocalization Time (ms)', 'Status'])

            for _ in range(max_tests):
                self.total_tests += 1

                x_true, y_true = (self.amcl_pose.x, self.amcl_pose.y) if self.amcl_pose else (0.0, 0.0)

                x_pose = x_true + random.uniform(-0.0, 1.0)
                y_pose = y_true + random.uniform(-0.0, 0.0)
                self.publish_initial_pose(x_pose, y_pose)
                rospy.sleep(2)

                x_goal = x_true + random.uniform(-0.0, 1.0)
                y_goal = y_true + random.uniform(-0.0, 0.0)

                start_time = time.time()
                self.send_goal(x_goal, y_goal)
                relocalization_time = (time.time() - start_time) * 1000

                delta_x = abs(x_true - x_pose)
                delta_y = abs(y_true - y_pose)

                status = 'Success' if relocalization_time < 40000 else 'Failure'

                writer.writerow([
                    self.total_tests, x_true, y_true, x_pose, y_pose, x_goal, y_goal,
                    delta_x, delta_y, self.covariance_x, self.covariance_y,
                    relocalization_time, status
                ])

                self.reset_position()
                rospy.sleep(1)

        self.monitoring_active = False

if __name__ == "__main__":
    tester = AMCLNavigationTest()
    tester.test_navigation(max_tests=10)
