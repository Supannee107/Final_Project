#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from tf2_geometry_msgs import PoseStamped
import math

# Variables to store poses
amcl_pose = None
base_link_pose = None

# Robot name in Gazebo
ROBOT_NAME = "/"

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
        rospy.loginfo("AMCL Pose: [x: %f, y: %f, z: %f]", amcl_pose.position.x, amcl_pose.position.y, amcl_pose.position.z)
        rospy.loginfo("Base Link Pose: [x: %f, y: %f, z: %f]", base_link_pose.position.x, base_link_pose.position.y, base_link_pose.position.z)
        rospy.loginfo("AMCL Pose vs Base Link Distance: %f", distance)
    else:
        rospy.loginfo("Waiting for poses to be initialized...")

def listener():
    rospy.init_node('pose_comparator', anonymous=True)

    # Subscribe to topics
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_pose_callback)
    rospy.Subscriber("/gazebo/model_states", ModelStates, ground_truth_callback)

    # Run comparison every 1 second
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        compare_poses()
        rate.sleep()

if __name__ == '__main__':
    listener()


