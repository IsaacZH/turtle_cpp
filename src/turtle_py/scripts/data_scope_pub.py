#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose
from std_msgs.msg import Float64
import math

group4_pose = Pose()
group4_2_pose = Pose()

def group4_pose_callback(msg):
    global group4_pose
    group4_pose = msg

def group4_2_pose_callback(msg):
    global group4_2_pose
    group4_2_pose = msg

def calculate_distance(pose1, pose2):
    return math.sqrt((pose1.x - pose2.x)**2 + (pose1.y - pose2.y)**2)

def main():
    rospy.init_node('data_scope_pub')
    rospy.Subscriber('/Group4/pose', Pose, group4_pose_callback)
    rospy.Subscriber('/Group4_2/pose', Pose, group4_2_pose_callback)
    distance_pub = rospy.Publisher('/turtle_distance', Float64, queue_size=10)
    orientation_pub = rospy.Publisher('/turtle_2_orientation', Float64, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        distance_msg = Float64()
        distance_msg.data = calculate_distance(group4_pose, group4_2_pose)
        distance_pub.publish(distance_msg)

        orientation_msg = Float64()
        orientation_msg.data = group4_2_pose.theta
        orientation_pub.publish(orientation_msg)

        rospy.loginfo("Group4_Pose - X: %f, Y: %f, Theta: %f    Distance: %f, Group4_2 theta: %f",
                      group4_pose.x, group4_pose.y, group4_pose.theta, distance_msg.data, group4_2_pose.theta)
        rate.sleep()

if __name__ == '__main__':
    main()