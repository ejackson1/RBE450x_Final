#!/usr/bin/env python3

import sys
import rospy
import actionlib
import moveit_commander

from arm_control.msg import MoveArmAction, MoveArmResult, MoveArmCartAction, MoveArmCartResult
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

joint_states = JointState()

# Tolerance for EE pose
GOAL_TOLERANCE = 0.001

# Action server that handles requests to send goals to MoveIt and relay waypoints to cFS
class MoveArmServer:
  def __init__(self, name):

    rospy.loginfo("Initializing action server and commanders...")
    self.move_server = actionlib.SimpleActionServer("move_arm", MoveArmAction, execute_cb=self.move_cb, auto_start=False)
    self.move_server.start()

    self.cart_server = actionlib.SimpleActionServer("move_arm_cart", MoveArmCartAction, execute_cb=self.cart_cb, auto_start=False)
    self.cart_server.start()

    self.robot = moveit_commander.RobotCommander()

    self.arm_group = moveit_commander.MoveGroupCommander("panda_arm")
    self.planning_scene = moveit_commander.PlanningSceneInterface()

    self.js_sub = rospy.Subscriber("/panda/joint_states", JointState, self.js_cb)
    self.left_finger_pub = rospy.Publisher("/panda/panda_finger1_controller/command", Float64, queue_size=1000)
    self.right_finger_pub = rospy.Publisher("/panda/panda_finger2_controller/command", Float64, queue_size=1000)

    rospy.sleep(2)

    rospy.loginfo("Connected to MoveGroup and Robot Commanders.")

  def publishFinger(self, isLeft, pos):
    if isLeft:
      self.left_finger_pub.publish(Float64(data=pos))
    else:
      self.right_finger_pub.publish(Float64(data=pos))

  def setGripper(self, open):
    left = self.robot.get_joint('panda_finger_joint1')
    right = self.robot.get_joint('panda_finger_joint2')

    cur_left = joint_states.position[joint_states.name.index("panda_finger_joint1")]
    cur_right = joint_states.position[joint_states.name.index("panda_finger_joint2")]

    print(f"Left current: {cur_left}\tRight current: {cur_right}")

    left_pose, right_pose = 0, 0

    if open:
      right_pose = right.max_bound()
      left_pose = right.max_bound()
    else:
      left_pose = left.min_bound() + 0.01
      right_pose = right.min_bound() + 0.01

    print(left_pose)
    print(right_pose)

    self.publishFinger(True, left_pose)
    self.publishFinger(False, right_pose)

  def js_cb(self, js):
    global joint_states
    joint_states = js

  def move_cb(self, goal):
    rospy.loginfo("Received move goal! Getting plan...")

    self.setGripper(goal.open)

    self.arm_group.set_pose_target(goal.ee_pose)
    self.arm_group.set_pose_reference_frame(goal.ee_pose.header.frame_id)
    self.arm_group.set_goal_position_tolerance(goal.tolerance)
      
    rospy.loginfo("Executing...")
    plan = self.arm_group.go(wait=True)

    self.arm_group.stop()
    success = True

    if(success):
      rospy.loginfo("Plan from move group successful.")
    else:
      rospy.logerr("Plan failed, most likely cause is the goal is outside the task space.")
      self.server.set_aborted(MoveArmResult(success))
      return

    # Once the trajectory has finished, succeed on the AS goal
    self.move_server.set_succeeded(MoveArmResult(success))

  def cart_cb(self, goal):
    rospy.loginfo("Received cartesian goal! Getting plan...")

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, _) = self.arm_group.compute_cartesian_path(
                                      goal.ee_poses,   # waypoints to follow
                                      0.01,            # eef_step
                                      0.0)             # jump_threshold

    self.arm_group.execute(plan, wait=True)

    # Once the trajectory has finished, succeed on the AS goal
    self.cart_server.set_succeeded(MoveArmResult(True))


if __name__ == "__main__":
  rospy.init_node("moveit_interface")
  args = rospy.myargv(argv=sys.argv)

  moveit_commander.roscpp_initialize(sys.argv)

  server = MoveArmServer(rospy.get_name())

  rospy.spin()
