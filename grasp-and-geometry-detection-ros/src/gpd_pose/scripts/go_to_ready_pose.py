#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import math
import rospy
import moveit_commander

def go_to_ready_pose():
    # Initialize ROS and MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("ur5e_go_to_ready", anonymous=True)

    # Create MoveGroupCommander for arm and gripper
    arm_group = moveit_commander.MoveGroupCommander("ur5e_arm")
    gripper_group = moveit_commander.MoveGroupCommander("ur5e_gripper")

    # Define ready pose (joint goals)
    ready_pose = {
        "shoulder_pan_joint": math.pi / 2,
        "shoulder_lift_joint": -math.pi * 0.75,
        "elbow_joint": math.pi / 2,
        "wrist_1_joint": -math.pi / 2,
        "wrist_2_joint": -math.pi / 2,
        "wrist_3_joint": math.pi
    }

    # Define gripper states
    gripper_joint = "gripper_robotiq_hande_joint_finger"
    gripper_open = 0.0
    gripper_closed = 0.020

    # --- Open Gripper ---
    rospy.loginfo("Opening gripper...")
    gripper_group.set_joint_value_target({gripper_joint: gripper_open})
    gripper_group.go(wait=True)
    gripper_group.stop()

    # --- Move to Ready Pose ---
    rospy.loginfo("Moving UR5e arm to ready pose...")
    arm_group.set_joint_value_target(ready_pose)

    # Plan returns a tuple: (success_flag, plan, time, error_code)
    plan_success, plan, _, _ = arm_group.plan()

    if not plan_success or len(plan.joint_trajectory.points) == 0:
        rospy.logerr("Planning failed.")
        return

    rospy.loginfo("Plan successful. Executing trajectory...")
    arm_group.execute(plan, wait=True)
    arm_group.stop()

    # Shut down MoveIt cleanly
    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    try:
        go_to_ready_pose()
    except rospy.ROSInterruptException:
        pass
