#!/usr/bin/env python3

import rospy
import actionlib
import csv
import os
import threading
import psutil
import time
from datetime import datetime
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler

class MoveBaseWithLogging:
    def __init__(self):
        rospy.init_node('move_base_logger')

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base server!")

        self.amcl_subscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        self.odom_subscriber = rospy.Subscriber('/velocity_controller/odom', Odometry, self.odom_callback)

        self.data_log = []
        self.logging = True
        self.lock = threading.Lock()

        self.csv_directory = "/home/supannee/Project/src/Final_Project/amcl_mobile_robot/result/Normal_Ob/5_meter_Ob/iter5/"
        self.csv_filename = os.path.join(self.csv_directory, "localization_data_amcl.csv")
        self.system_log_filename = os.path.join(self.csv_directory, "system_usage.csv")

        if not os.path.exists(self.csv_directory):
            os.makedirs(self.csv_directory)

        if not os.path.exists(self.csv_filename):
            with open(self.csv_filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["amcl_time", "amcl_X", "amcl_Y", "Odom_time", "Odom_X", "Odom_Y", "Distance"])

        if not os.path.exists(self.system_log_filename):
            with open(self.system_log_filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['Timestamp', 'CPU Usage (%)', 'RAM Usage (%)'])

        self.latest_amcl = None
        self.latest_odom = None
        self.amcl_time = None
        self.odom_time = None

        self.logging_thread = threading.Thread(target=self.log_position)
        self.logging_thread.start()

        self.system_monitor_thread = threading.Thread(target=self.log_system_usage)
        self.system_monitor_thread.start()

        rospy.on_shutdown(self.cleanup)

    def amcl_callback(self, msg):
        self.latest_amcl = msg.pose
        self.amcl_time = msg.header.stamp.to_sec()

    def odom_callback(self, msg):
        self.latest_odom = msg.pose.pose
        self.odom_time = msg.header.stamp.to_sec()

    def calculate_distance(self, pose1, pose2):
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        return (dx**2 + dy**2) ** 0.5

    def log_position(self):
        rate = rospy.Rate(10)
        while self.logging and not rospy.is_shutdown():
            if self.latest_amcl and self.latest_odom:
                with self.lock:
                    amcl_x = self.latest_amcl.pose.position.x  # FIXED
                    amcl_y = self.latest_amcl.pose.position.y  # FIXED
                    odom_x = self.latest_odom.position.x
                    odom_y = self.latest_odom.position.y
                    distance = self.calculate_distance(self.latest_amcl.pose, self.latest_odom)  # FIXED

                    with open(self.csv_filename, mode='a', newline='') as file:
                        writer = csv.writer(file)
                        writer.writerow([self.amcl_time, amcl_x, amcl_y, self.odom_time, odom_x, odom_y, distance])
            rate.sleep()


    def log_system_usage(self):
        while self.logging and not rospy.is_shutdown():
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            cpu_usage = psutil.cpu_percent(interval=0.1)
            ram_usage = psutil.virtual_memory().percent
            with open(self.system_log_filename, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([timestamp, cpu_usage, ram_usage])
            time.sleep(0.1)

    def move_to_goal(self, x, y, theta):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, theta)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        rospy.loginfo(f"Sending goal: x={x}, y={y}, theta={theta}")
        self.client.send_goal(goal)
        self.client.wait_for_result()

        if self.client.get_result():
            rospy.loginfo("Goal reached successfully!")
        else:
            rospy.logwarn("Failed to reach goal.")

        self.logging = False

    def cleanup(self):
        rospy.loginfo("Shutting down logging threads...")
        self.logging = False
        self.logging_thread.join()
        self.system_monitor_thread.join()
        rospy.loginfo("Shutdown complete.")

if __name__ == '__main__':
    try:
        mover = MoveBaseWithLogging()
        mover.move_to_goal(5.0, 0.0, 0.0)
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted.")