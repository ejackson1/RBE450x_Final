#!/usr/bin/env python3

import sys
import rospy
import actionlib
import moveit_commander

from arm_control.msg import MoveArmAction, MoveArmResult

# Tolerance for EE pose
GOAL_TOLERANCE = 0.001

# Action server that handles requests to send goals to MoveIt and relay waypoints to cFS
class MoveArmServer:
  def __init__(self, name):

    rospy.loginfo("Initializing action server and commanders...")
    self.action_name = name
    self.server = actionlib.SimpleActionServer(rospy.get_name(), MoveArmAction, execute_cb=self.as_cb, auto_start=False)
    self.server.start()

    self.robot = moveit_commander.RobotCommander()

    self.group = moveit_commander.MoveGroupCommander("panda_arm")
    self.planning_scene = moveit_commander.PlanningSceneInterface()

    rospy.sleep(2)

    rospy.loginfo("Connected to MoveGroup and Robot Commanders.")

  def as_cb(self, goal):
    rospy.loginfo("Received goal! Getting plan...")

    self.group.set_pose_target(goal.ee_pose)
    self.group.set_pose_reference_frame(goal.ee_pose.header.frame_id)
    self.group.set_goal_position_tolerance(goal.tolerance)
      
    rospy.loginfo("Executing...")
    plan = self.group.go(wait=True)

    self.group.stop()
    success = True

    if(success):
      rospy.loginfo("Plan from move group successful.")
    else:
      rospy.logerr("Plan failed, most likely cause is the goal is outside the task space.")
      self.server.set_aborted(MoveArmResult(success))
      return

    # Once the trajectory has finished, succeed on the AS goal
    self.server.set_succeeded(MoveArmResult(success))


if __name__ == "__main__":
  rospy.init_node("moveit_interface")
  args = rospy.myargv(argv=sys.argv)

  moveit_commander.roscpp_initialize(sys.argv)

  server = MoveArmServer(rospy.get_name())

  rospy.spin()
