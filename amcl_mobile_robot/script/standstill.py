#!/usr/bin/env python3

import rospy
import csv
import os
import time
import psutil
from datetime import datetime
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped

import threading

class StationaryLogging:
    def __init__(self):
        rospy.init_node('stationary_logger')
        
        # Subscriber เพื่อรับข้อมูลตำแหน่งจาก /amcl_pose และ /odom
        self.amcl_subscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        self.odom_subscriber = rospy.Subscriber('/velocity_controller/odom', Odometry, self.odom_callback)
        
        # ตัวแปรเก็บค่าตำแหน่ง
        self.logging = True  # ควบคุมการบันทึก

        # กำหนดพาธสำหรับบันทึกไฟล์ CSV
        self.csv_directory = os.path.expanduser("/home/supannee/Project/src/Final_Project/amcl_mobile_robot/result/Narrow_Map/Standstill/")
        self.csv_filename = os.path.join(self.csv_directory, "localization_stationary_data.csv")
        self.system_usage_filename = os.path.join(self.csv_directory, "system_usage_Stationary.csv")

        # ตรวจสอบว่าไดเรกทอรีมีอยู่หรือไม่ ถ้าไม่มีให้สร้างใหม่
        if not os.path.exists(self.csv_directory):
            os.makedirs(self.csv_directory)

        # สร้างไฟล์ CSV หากยังไม่มี
        with open(self.csv_filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["amcl_Timestamp", "amcl_X", "amcl_Y", "Odom_Timestamp", "Odom_X", "Odom_Y"])
        
        with open(self.system_usage_filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Timestamp', 'CPU Usage (%)', 'RAM Usage (%)'])

        # ตัวแปรเก็บค่าล่าสุด
        self.latest_amcl = None
        self.latest_odom = None
        self.latest_amcl_time = None
        self.latest_odom_time = None

        # ใช้ Thread สำหรับบันทึกตำแหน่งและข้อมูลระบบ
        self.logging_thread = threading.Thread(target=self.log_position)
        self.system_usage_thread = threading.Thread(target=self.log_system_usage)
        self.logging_thread.start()
        self.system_usage_thread.start()

    def amcl_callback(self, msg):
        """ Callback สำหรับรับค่าตำแหน่งจาก amcl """
        self.latest_amcl = msg.pose  # ดึงค่า pose เท่านั้น
        self.latest_amcl_time = msg.header.stamp.to_sec()
        rospy.loginfo(f"amcl Pose Updated: X={msg.pose.position.x}, Y={msg.pose.position.y}")

    def odom_callback(self, msg):
        """ Callback สำหรับรับค่าตำแหน่งจาก Odom (Ground Truth) """
        self.latest_odom = msg.pose.pose  # ดึงค่า pose เท่านั้น
        self.latest_odom_time = msg.header.stamp.to_sec()
        rospy.loginfo(f"Odom Pose Updated: X={msg.pose.pose.position.x}, Y={msg.pose.pose.position.y}")

    def log_position(self):
        """ บันทึกค่าตำแหน่งทุก 0.1 วินาที เป็นเวลา 2 นาที """
        rospy.loginfo("Waiting for amcl and Odom data before logging...")
        while not self.latest_amcl or not self.latest_odom:
            rospy.sleep(0.1)

        rospy.loginfo("amcl and Odom data received. Starting logging...")
        start_time = rospy.Time.now().to_sec()
        duration = 120  # 2 นาที (120 วินาที)
        rate = rospy.Rate(10)  # 10 Hz (0.1 วินาที)

        while self.logging and not rospy.is_shutdown():
            current_time = rospy.Time.now().to_sec()
            if (current_time - start_time) >= duration:
                rospy.loginfo("Logging duration reached. Stopping...")
                break

            if self.latest_amcl and self.latest_odom:
                amcl_time = self.latest_amcl_time
                amcl_x = self.latest_amcl.pose.position.x
                amcl_y = self.latest_amcl.pose.position.y

                odom_time = self.latest_odom_time
                odom_x = self.latest_odom.position.x
                odom_y = self.latest_odom.position.y

                with open(self.csv_filename, mode='a', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow([amcl_time, amcl_x, amcl_y, odom_time, odom_x, odom_y])

                rospy.loginfo(f"✅ Data written to CSV: amcl=({amcl_x}, {amcl_y}) at {amcl_time} | ODOM=({odom_x}, {odom_y}) at {odom_time}")

            rate.sleep()

        self.logging = False
        rospy.loginfo("Logging complete.")


    
    def log_system_usage(self):
        """ บันทึกค่าการใช้งาน CPU และ RAM ทุก 0.1 วินาที """
        rate = rospy.Rate(10)  # 10 Hz (0.1 วินาที)
        start_time = rospy.Time.now().to_sec()
        duration = 120  # 2 นาที
        
        while self.logging and not rospy.is_shutdown():
            current_time = rospy.Time.now().to_sec()
            if (current_time - start_time) >= duration:
                break
            
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            cpu_usage = psutil.cpu_percent(interval=None)
            ram_usage = psutil.virtual_memory().percent
            
            with open(self.system_usage_filename, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([timestamp, cpu_usage, ram_usage])
            
            rospy.loginfo(f"📊 System Usage: {timestamp} | CPU: {cpu_usage}% | RAM: {ram_usage}%")
            
            rate.sleep()

if __name__ == '__main__':
    try:
        logger = StationaryLogging()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Logging interrupted.")
