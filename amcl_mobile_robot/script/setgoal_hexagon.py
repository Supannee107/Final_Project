#!/usr/bin/env python3
import rospy
import csv
import os
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from tf2_geometry_msgs import PoseStamped
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped 

# Variables to store poses
amcl_pose = None
base_link_pose = None

# Robot name in Gazebo
ROBOT_NAME = "/"

# CSV file path
CSV_FILE = "hexagon(Distance between AMCL and Base Link).csv"

# Initialize CSV file with headers if not exists
if not os.path.exists(CSV_FILE):
    with open(CSV_FILE, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Timestamp", "AMCL(X)", "AMCL(Y)", "Base Link(X)", "Base Link(Y)", "Distance between AMCL and Base Link"])

# Callback function to get amcl_pose
def amcl_pose_callback(msg):
    global amcl_pose
    amcl_pose = msg.pose.pose

# Callback function to get ground_truth (base_link in Gazebo)
def ground_truth_callback(msg):
    global base_link_pose
    try:
        # Find index of the robot in Gazebo model states
        index = msg.name.index(ROBOT_NAME)
        base_link_pose = msg.pose[index]
    except ValueError:
        rospy.logwarn(f"Model {ROBOT_NAME} not found in Gazebo model states.")

# Function to calculate Euclidean distance
def calculate_distance(pose1, pose2):
    dx = pose1.position.x - pose2.position.x
    dy = pose1.position.y - pose2.position.y
    return math.sqrt(dx**2 + dy**2)

def compare_poses():
    if amcl_pose and base_link_pose:
        distance = calculate_distance(amcl_pose, base_link_pose)

        # Get timestamp
        timestamp = rospy.get_time()

        # Calculate yaw for both poses
        yaw_amcl = euler_from_quaternion([
            amcl_pose.orientation.x,
            amcl_pose.orientation.y,
            amcl_pose.orientation.z,
            amcl_pose.orientation.w
        ])[2]
        yaw_base_link = euler_from_quaternion([
            base_link_pose.orientation.x,
            base_link_pose.orientation.y,
            base_link_pose.orientation.z,
            base_link_pose.orientation.w
        ])[2]

        # Log the results
        rospy.loginfo("AMCL Pose: [x: %f, y: %f]", amcl_pose.position.x, amcl_pose.position.y)
        rospy.loginfo("Base Link Pose: [x: %f, y: %f]", base_link_pose.position.x, base_link_pose.position.y)
        rospy.loginfo("AMCL Pose vs Base Link Distance: %f", distance)

        # Write data to CSV
        with open(CSV_FILE, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                timestamp,
                amcl_pose.position.x, amcl_pose.position.y,
                base_link_pose.position.x, base_link_pose.position.y,
                distance
            ])
    else:
        rospy.loginfo("Waiting for poses to be initialized...")



# Set and send pose goal
def set_pose_goal(x, y, z, qx, qy, qz, qw):
    """
    Sends a pose goal to the move_base node.

    :param x: X position of the goal
    :param y: Y position of the goal
    :param z: Z position of the goal (usually 0 for 2D navigation)
    :param qx: Quaternion X component
    :param qy: Quaternion Y component
    :param qz: Quaternion Z component
    :param qw: Quaternion W component
    """
    # Publisher to the move_base SimpleGoal topic
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.sleep(1)  # Allow some time for the publisher to set up

    # Create a PoseStamped message
    goal = PoseStamped()
    goal.header.frame_id = "map"  # Reference frame
    goal.header.stamp = rospy.Time.now()

    # Set the position
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = z

    # Set the orientation (as a quaternion)
    goal.pose.orientation.x = qx
    goal.pose.orientation.y = qy
    goal.pose.orientation.z = qz
    goal.pose.orientation.w = qw

    # Publish the goal
    rospy.loginfo("Sending pose goal: %s", goal)
    pub.publish(goal)

def main():
    rospy.init_node('set_pose_and_compare', anonymous=True)

    # Subscribe to topic
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_pose_callback)
    rospy.Subscriber("/gazebo/model_states", ModelStates, ground_truth_callback)
    # Set and send the pose goal
    rospy.loginfo("Setting pose goal...")
    set_pose_goal(
        x=0.0, y=-3.2, z=0.0,
        qx=0.0, qy=0.0, qz=0.0, qw=1.0
    )

    # Run comparison at 1 Hz

    rospy.loginfo("Comparing poses while robot moves...")
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        compare_poses()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupted.")
