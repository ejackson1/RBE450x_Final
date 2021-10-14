#!/usr/bin/env python3

import rospy
import actionlib

from sensor_msgs.msg import PointCloud2, PointField
from arm_control.msg import MoveArmGoal, MoveArmAction
from geometry_msgs.msg import PoseStamped

from copy import deepcopy

# Global version of the last received point cloud
last_pc = None

# Global version of the SimpleActionClient for move_arm
move_client = None

# Updates the last_pc global object with the received point cloud
def pc_cb(pc_msg):
    global last_pc

    last_pc = pc_msg

def combinePointCloud(pc1, pc2):

    if (pc1.height != pc2.height or
        pc1.point_step != pc2.point_step):
        rospy.logerr("Point cloud metadata does not match, cannot merge")

        print(pc1.point_step)
        print(pc2.point_step)

        print(pc1.row_step)
        print(pc2.row_step)
        return
    
    out = deepcopy(pc1)

    out.data += pc2.data
    out.width += pc2.width
    
    return out

# Move the robot arm to set position and orientation. Takes global coordinates.
def travel2Pose(poseStamped):
        
    rospy.loginfo("Sending to Position!")
    
    # Tell arm to move towards pose Stamped value
    goal = MoveArmGoal()
    goal.ee_pose = poseStamped
    goal.ee_pose.header.frame_id = "world"
    goal.ee_pose.header.stamp = rospy.Time.now()

    goal.tolerance = 0.005

    move_client.send_goal_and_wait(goal)

if __name__ == "__main__":
    rospy.init_node("combine_point_clouds")

    pc_sub = rospy.Subscriber("/segmented", PointCloud2, pc_cb, queue_size=10)
    move_client = actionlib.SimpleActionClient('move_arm', MoveArmAction)

    pc_pub = rospy.Publisher("/pc_merged", PointCloud2, queue_size=10, latch=True)

    rospy.sleep(2.0)

    # 1. Go to start pose
    rospy.loginfo("Going to 1st pose")
    somePose = PoseStamped()
    somePose.pose.position.x = 0.477343
    somePose.pose.position.y = 0.00173367
    somePose.pose.position.z = 0.659652
    somePose.pose.orientation.x = 0.98356
    somePose.pose.orientation.y = 0.00474719
    somePose.pose.orientation.z = 0.18052
    somePose.pose.orientation.w = -0.000335126
    travel2Pose(somePose)
    rospy.sleep(3.5)

    # 2. Capture point cloud from segmenter
    pc1 = last_pc

    # 3. Go to a different pose
    rospy.loginfo("Going to 2nd pose")
    someOtherPose = PoseStamped()
    someOtherPose.pose.position.x = 0.555744
    someOtherPose.pose.position.y = -0.210957
    someOtherPose.pose.position.z = 0.673579
    someOtherPose.pose.orientation.x = 0.965545
    someOtherPose.pose.orientation.y = 0.0400095
    someOtherPose.pose.orientation.z = 0.175197
    someOtherPose.pose.orientation.w = -0.188226

    travel2Pose(someOtherPose)
    rospy.sleep(3.5)

    # 4. Save point cloud from segmenter
    pc2 = last_pc

    # 5. Go to a third pose
    rospy.loginfo("Going to 3rd pose")
    thirdPose = PoseStamped()
    thirdPose.pose.position.x = 0.695556
    thirdPose.pose.position.y = -0.0314852
    thirdPose.pose.position.z = 0.679568
    thirdPose.pose.orientation.x = 0.98981
    thirdPose.pose.orientation.y = 0.110247
    thirdPose.pose.orientation.z = 0.0709012
    thirdPose.pose.orientation.w = -0.05563

    travel2Pose(thirdPose)
    rospy.sleep(3.5)

    # 6. Save point cloud
    pc3 = last_pc

    # 7. Combine 3 sensor_msgs::PointCloud2 ROS messages together
    rospy.loginfo("Merging point clouds")
    partial = combinePointCloud(pc1, pc2)
    merged  = combinePointCloud(partial, pc3)

    pc_pub.publish(merged)

    rospy.loginfo("Published merged message.")