#!/usr/bin/env python3

import rospy
import actionlib
import csv
import os
import time
from move_base_msgs.msg import MoveBaseAction
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import threading

class MoveBaseWithLogging:
    def __init__(self):
        rospy.init_node('move_base_logger')

        # Publisher สำหรับควบคุมการหมุนของหุ่นยนต์
        self.cmd_vel_publisher = rospy.Publisher('/velocity_controller/cmd_vel', Twist, queue_size=10)
        
        # Subscriber เพื่อรับข้อมูลตำแหน่งจาก /amcl_pose และ /odom
        self.amcl_subscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        self.odom_subscriber = rospy.Subscriber('/velocity_controller/odom', Odometry, self.odom_callback)

        # ตัวแปรเก็บค่าตำแหน่ง
        self.data_log = []  # เก็บค่าทั้ง amcl_pose และ odom
        self.logging = True  # ควบคุมการบันทึก

        # กำหนดพาธสำหรับบันทึกไฟล์ CSV
        self.csv_directory = os.path.expanduser("/home/supannee/Project/src/Final_Project/amcl_mobile_robot/result/")
        self.csv_filename = os.path.join(self.csv_directory, "localization_rotation_data.csv")

        # ตรวจสอบว่าไดเรกทอรีมีอยู่หรือไม่ ถ้าไม่มีให้สร้างใหม่
        if not os.path.exists(self.csv_directory):
            os.makedirs(self.csv_directory)

        # สร้างไฟล์ CSV หากยังไม่มี
        with open(self.csv_filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Timestamp", "amcl_X", "amcl_Y", "Odom_X", "Odom_Y"])

        # ตัวแปรเก็บค่าล่าสุด
        self.latest_amcl = None
        self.latest_odom = None

        # ใช้ Thread สำหรับบันทึกตำแหน่ง
        self.logging_thread = threading.Thread(target=self.log_position)
        self.logging_thread.start()

    def amcl_callback(self, msg):
        """ Callback สำหรับรับค่าตำแหน่งจาก amcl """
        self.latest_amcl = msg.pose.pose  # ดึงค่า pose เท่านั้น
        rospy.loginfo(f"amcl Pose Updated: X={msg.pose.pose.position.x}, Y={msg.pose.pose.position.y}")

    def odom_callback(self, msg):
        """ Callback สำหรับรับค่าตำแหน่งจาก Odom (Ground Truth) """
        self.latest_odom = msg.pose.pose  # ดึงค่า pose เท่านั้น
        rospy.loginfo(f"Odom Pose Updated: X={msg.pose.pose.position.x}, Y={msg.pose.pose.position.y}")

    def log_position(self):
        """ บันทึกค่าตำแหน่งทุก 0.1 วินาที """
        rospy.loginfo("Waiting for AMCL and Odom data before logging...")
        while not self.latest_amcl or not self.latest_odom:
            rospy.sleep(0.1)
        
        rospy.loginfo("AMCL and Odom data received. Starting logging...")
        rate = rospy.Rate(10)  # 10 Hz (0.1 วินาที)
        while self.logging and not rospy.is_shutdown():
            rospy.sleep(0.1)
            
            if self.latest_amcl and self.latest_odom:
                # ดึงข้อมูลจาก amcl_pose และ odom
                timestamp = rospy.get_time()
                amcl_x = self.latest_amcl.position.x
                amcl_y = self.latest_amcl.position.y
                odom_x = self.latest_odom.position.x
                odom_y = self.latest_odom.position.y

                # บันทึกข้อมูลลง list
                self.data_log.append([timestamp, amcl_x, amcl_y, odom_x, odom_y])
                rospy.loginfo(f"Logging Data: amcl=({amcl_x}, {amcl_y}) | ODOM=({odom_x}, {odom_y})")

            rate.sleep()

    def rotate_robot(self, duration=60):
        """ Rotate the robot in place for a given duration (seconds). """
        rospy.loginfo("Starting rotation...")
        twist = Twist()
        twist.angular.z = 0.5  # Set angular speed

        rate = rospy.Rate(50)  # Publish more frequently
        start_time = rospy.Time.now().to_sec()

        while (rospy.Time.now().to_sec() - start_time) < duration and not rospy.is_shutdown():
            self.cmd_vel_publisher.publish(twist)
            rate.sleep()
        
        # Stop the rotation
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        rospy.loginfo("Rotation complete.")

        # Stop logging
        self.logging = False

        # Save all data at the end
        self.save_data_to_csv()

    def save_data_to_csv(self):
        """ Save collected data to CSV """
        rospy.loginfo(f"Saving {len(self.data_log)} entries to {self.csv_filename}")

        if len(self.data_log) == 0:
            rospy.logwarn("No data collected! Check if logging is working correctly.")
            return

        with open(self.csv_filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerows(self.data_log)

        rospy.loginfo(f"CSV file saved successfully with {len(self.data_log)} entries.")


if __name__ == '__main__':
    try:
        mover = MoveBaseWithLogging()
        mover.rotate_robot()  # หมุนอยู่กับที่เป็นเวลา 1 นาที
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted.")
