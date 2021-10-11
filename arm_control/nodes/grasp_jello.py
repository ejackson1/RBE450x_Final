#!/usr/bin/env python3

import tf
import rospy
import actionlib

from arm_control.msg import MoveArmAction, MoveArmGoal, MoveArmCartAction, MoveArmCartGoal
from geometry_msgs.msg import PoseStamped

if __name__ == "__main__":
    rospy.init_node("example_moveit_client")
    
    move_client = actionlib.SimpleActionClient('move_arm', MoveArmAction)
    cart_client = actionlib.SimpleActionClient('move_arm_cart', MoveArmCartAction)

    move_client.wait_for_server()
    cart_client.wait_for_server()

    ### Approach
    rospy.loginfo("Sending to approach")
    approach = PoseStamped()

    approach.pose.position.x = 0.65
    approach.pose.position.y = -0.01
    approach.pose.position.z = 0.7

    quaternion = tf.transformations.quaternion_from_euler(3.1, 0, 1.34)

    approach.pose.orientation.x = quaternion[0]
    approach.pose.orientation.y = quaternion[1]
    approach.pose.orientation.z = quaternion[2]
    approach.pose.orientation.w = quaternion[3]

    goal = MoveArmGoal()

    goal.ee_pose = approach
    goal.ee_pose.header.frame_id = "world"
    goal.ee_pose.header.stamp = rospy.Time.now()

    goal.tolerance = 0.005

    move_client.send_goal_and_wait(goal)

    ### Open gripper
    rospy.loginfo("Opening gripper")

    goal.open = True

    move_client.send_goal_and_wait(goal)

    ### Cartesian path
    rospy.loginfo("Gripper opened. Approaching along cartesian path")

    goal.ee_pose.pose.position.z -= 0.12

    cart_goal = MoveArmCartGoal()
    cart_goal.ee_poses = [goal.ee_pose.pose]

    cart_client.send_goal(cart_goal)

    cart_client.wait_for_result()

    ### Grip
    rospy.loginfo("Finished cart traj. Gripping...")

    goal.open = False

    move_client.send_goal_and_wait(goal)
    rospy.sleep(1)

    ### Undo cartesian path
    rospy.loginfo("Attempting to move away")

    goal.ee_pose.pose.position.z += 0.12

    cart_goal = MoveArmCartGoal()
    cart_goal.ee_poses = [goal.ee_pose.pose]

    cart_client.send_goal(cart_goal)

    cart_client.wait_for_result()