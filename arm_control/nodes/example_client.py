#!/usr/bin/env python3

import tf
import rospy
import actionlib

from arm_control.msg import MoveArmAction, MoveArmGoal
from geometry_msgs.msg import PoseStamped

if __name__ == "__main__":
    rospy.init_node("example_moveit_client")
    
    client = actionlib.SimpleActionClient('moveit_interface', MoveArmAction)

    client.wait_for_server()

    x = float(input("X: "))
    y = float(input("Y: "))
    z = float(input("Z: "))

    roll = float(input("Roll: "))
    pitch = float(input("Pitch: "))
    yaw = float(input("Yaw: "))

    goal = MoveArmGoal()

    goal.ee_pose = PoseStamped()
    goal.ee_pose.header.frame_id = "world"
    goal.ee_pose.header.stamp = rospy.Time.now()

    goal.ee_pose.pose.position.x = x
    goal.ee_pose.pose.position.y = y
    goal.ee_pose.pose.position.z = z

    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

    goal.ee_pose.pose.orientation.x = quaternion[0]
    goal.ee_pose.pose.orientation.y = quaternion[1]
    goal.ee_pose.pose.orientation.z = quaternion[2]
    goal.ee_pose.pose.orientation.w = quaternion[3]

    goal.tolerance = 0.01

    client.send_goal(goal)

    rospy.loginfo("Sent goal")

    client.wait_for_result()

    rospy.loginfo("Goal complete!")