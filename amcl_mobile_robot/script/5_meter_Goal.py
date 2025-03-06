#!/usr/bin/env python3

import rospy
import actionlib
import csv
import os
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import threading

class MoveBaseWithLogging:
    def __init__(self):
        rospy.init_node('move_base_logger')

        # สร้าง Client สำหรับ move_base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base server!")

        # Subscriber เพื่อรับข้อมูลตำแหน่งจาก /amcl_pose และ /odom
        self.amcl_subscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        self.odom_subscriber = rospy.Subscriber('/velocity_controller/odom', Odometry, self.odom_callback)

        # ตัวแปรเก็บค่าตำแหน่ง
        self.data_log = []  # เก็บค่าทั้ง amcl_pose และ odom
        self.logging = True  # ควบคุมการบันทึก

        # กำหนดพาธสำหรับบันทึกไฟล์ CSV
        self.csv_directory = os.path.expanduser("~/Project/src/Final_Project/amcl_mobile_robot/result/")
        self.csv_filename = os.path.join(self.csv_directory, "localization_data_narraw1.csv")

        # ตรวจสอบว่าไดเรกทอรีมีอยู่หรือไม่ ถ้าไม่มีให้สร้างใหม่
        if not os.path.exists(self.csv_directory):
            os.makedirs(self.csv_directory)

        # ตัวแปรเก็บค่าล่าสุด
        self.latest_amcl = None
        self.latest_odom = None

        # ใช้ Thread สำหรับบันทึกตำแหน่ง
        self.logging_thread = threading.Thread(target=self.log_position)
        self.logging_thread.start()

    def amcl_callback(self, msg):
        """ Callback สำหรับรับค่าตำแหน่งจาก AMCL """
        self.latest_amcl = msg
        rospy.loginfo(f"AMCL Pose Updated: X={msg.pose.pose.position.x}, Y={msg.pose.pose.position.y}")

    def odom_callback(self, msg):
        """ Callback สำหรับรับค่าตำแหน่งจาก Odom (Ground Truth) """
        self.latest_odom = msg
        rospy.loginfo(f"Odom Pose Updated: X={msg.pose.pose.position.x}, Y={msg.pose.pose.position.y}")

    def log_position(self):
        """ บันทึกค่าตำแหน่งทุก 0.1 วินาที """
        rate = rospy.Rate(10)  # 10 Hz (0.1 วินาที)
        while self.logging and not rospy.is_shutdown():
            rospy.sleep(0.1)

            if self.latest_amcl and self.latest_odom:
                # ดึงข้อมูลจาก amcl_pose
                amcl_x = self.latest_amcl.pose.pose.position.x
                amcl_y = self.latest_amcl.pose.pose.position.y
                amcl_time = self.latest_amcl.header.stamp.to_sec()

                # ดึงข้อมูลจาก odom (ground truth)
                odom_x = self.latest_odom.pose.pose.position.x
                odom_y = self.latest_odom.pose.pose.position.y
                odom_time = self.latest_odom.header.stamp.to_sec()

                # บันทึกข้อมูลลง list
                self.data_log.append([amcl_time, amcl_x, amcl_y, odom_time, odom_x, odom_y])
                rospy.loginfo(f"Logging Data: AMCL=({amcl_x}, {amcl_y}) | ODOM=({odom_x}, {odom_y})")

            rate.sleep()

    def move_to_goal(self, x, y, theta):
        """ ส่งคำสั่งให้หุ่นยนต์เคลื่อนที่ไปยังจุดหมาย """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # กำหนดตำแหน่งของจุดหมาย
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0  # หมุนที่มุม 0 องศา

        rospy.loginfo(f"Sending goal: x={x}, y={y}, theta={theta}")
        self.client.send_goal(goal)

        # รอจนกว่าหุ่นยนต์จะไปถึงจุดหมาย
        self.client.wait_for_result()

        if self.client.get_result():
            rospy.loginfo("Goal reached successfully!")
        else:
            rospy.logwarn("Failed to reach goal.")

        # หยุดการบันทึกและบันทึกลงไฟล์ CSV
        self.logging = False
        self.save_data_to_csv()

    def save_data_to_csv(self):
        """ บันทึกค่าตำแหน่งลงไฟล์ CSV """
        rospy.loginfo(f"Saving path data to {self.csv_filename}")

        if len(self.data_log) == 0:
            rospy.logwarn("No data collected! Check if /amcl_pose and /odom are publishing.")
            return

        with open(self.csv_filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["AMCL_Time", "AMCL_X", "AMCL_Y", "Odom_Time", "Odom_X", "Odom_Y"])
            writer.writerows(self.data_log)

        rospy.loginfo(f"Path data saved successfully: {len(self.data_log)} entries.")

if __name__ == '__main__':
    try:
        mover = MoveBaseWithLogging()
        mover.move_to_goal(5.0, 0.0, 0.0)  # ส่ง Goal ไปยัง (5,0,0)
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted.")
