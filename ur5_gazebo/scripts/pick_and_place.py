#!/usr/bin/env python3
import rospy
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler, quaternion_multiply
import copy
import time

if __name__ == "__main__":
    rospy.init_node("move_group_interface_tutorial", anonymous=True)

    # Initialize moveit_commander
    robot = RobotCommander()
    scene = PlanningSceneInterface()
    move_group_arm = MoveGroupCommander("ur5_arm")
    move_group_gripper = MoveGroupCommander("gripper")

    rospy.loginfo("Available Planning Groups:")
    rospy.loginfo(robot.get_group_names())

    # 1. Move to home position
    move_group_arm.set_named_target("home")
    move_group_arm.go(wait=True)
    move_group_arm.stop()
    move_group_arm.clear_pose_targets()

    # 2. Place the TCP above the blue box
    current_pose = move_group_arm.get_current_pose().pose
    target_pose1 = copy.deepcopy(current_pose)
    target_pose1.position.x += 0.0
    target_pose1.position.y -= 0.40
    target_pose1.position.z -= 0.35
    move_group_arm.set_pose_target(target_pose1)
    move_group_arm.go(wait=True)
    move_group_arm.stop()
    move_group_arm.clear_pose_targets()

    # 4. Close the gripper
    move_group_gripper.set_named_target("gripper_open")
    move_group_gripper.go(wait=True)
    move_group_gripper.stop()
    move_group_gripper.clear_pose_targets()

    # 3. Move the TCP close to the object
    target_pose1.position.z -= 0.09
    move_group_arm.set_pose_target(target_pose1)
    move_group_arm.go(wait=True)
    move_group_arm.stop()
    move_group_arm.clear_pose_targets()
    
    # 4. Close the gripper
    move_group_gripper.set_named_target("gripper_close")
    move_group_gripper.go(wait=True)
    move_group_gripper.stop()
    move_group_gripper.clear_pose_targets()

    # 5. Move the TCP above the plate
    target_pose1.position.z += 0.2
    move_group_arm.set_pose_target(target_pose1)
    move_group_arm.go(wait=True)
    move_group_gripper.stop()
    move_group_gripper.clear_pose_targets()

    # 6. Move the TCP above the plate
    target_pose1.position.y += 0.6
    move_group_arm.set_pose_target(target_pose1)
    move_group_arm.go(wait=True)
    move_group_gripper.stop()
    move_group_gripper.clear_pose_targets()

    # 7. Lower the TCP above the plate
    target_pose1.position.z -= 0.10
    move_group_arm.set_pose_target(target_pose1)
    move_group_arm.go(wait=True)
    move_group_gripper.stop()
    move_group_gripper.clear_pose_targets()

    # 8. Open the gripper
    move_group_gripper.set_named_target("gripper_open")
    move_group_gripper.go(wait=True)
    move_group_gripper.stop()
    move_group_gripper.clear_pose_targets()
