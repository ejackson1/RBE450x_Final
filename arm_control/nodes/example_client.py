#!/usr/bin/env python3

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

    goal = MoveArmGoal()

    goal.ee_pose = PoseStamped()
    goal.ee_pose.header.frame_id = "world"
    goal.ee_pose.header.stamp = rospy.Time.now()

    goal.ee_pose.pose.position.x = x
    goal.ee_pose.pose.position.y = y
    goal.ee_pose.pose.position.z = z

    goal.tolerance = 0.05

    client.send_goal(goal)

    rospy.log_info("Sent goal")

    client.wait_for_result()

    rospy.log_info("Goal complete!")