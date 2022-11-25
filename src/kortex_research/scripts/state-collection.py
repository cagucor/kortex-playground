#!/usr/bin/env python3

import sys
import time
from numpy._typing import NDArray
from numpy.lib import math
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import e, pi
from std_srvs.srv import Empty

import numpy as np
import pandas as pd
import random

# import csv


class StateCollector(object):
    """StateCollector"""

    def __init__(self):

        # Initialize the node
        super(StateCollector, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("state_collector")

        self.joint_shift = 0.1
        self.tolerance = 0.005
        # Note: actual limits are inf for joints 1,3,5,7
        self.actuator_limits = [
            math.pi - 2 * self.joint_shift,
            2.192 - 2 * self.joint_shift,
            math.pi - 2 * self.joint_shift,
            2.56 - 2 * self.joint_shift,
            math.pi - 2 * self.joint_shift,
            2.04 - 2 * self.joint_shift,
            math.pi - 2 * self.joint_shift,
        ]

        self.data = []

        try:
            self.is_gripper_present = rospy.get_param(
                rospy.get_namespace() + "is_gripper_present", False
            )
            if self.is_gripper_present:
                gripper_joint_names = rospy.get_param(
                    rospy.get_namespace() + "gripper_joint_names", []
                )
                self.gripper_joint_name = gripper_joint_names[0]
            else:
                self.gripper_joint_name = ""
            self.degrees_of_freedom = rospy.get_param(
                rospy.get_namespace() + "degrees_of_freedom", 7
            )

            # Create the MoveItInterface necessary objects
            arm_group_name = "arm"
            self.robot = moveit_commander.RobotCommander("robot_description")
            self.scene = moveit_commander.PlanningSceneInterface(
                ns=rospy.get_namespace()
            )
            self.arm_group = moveit_commander.MoveGroupCommander(
                arm_group_name, ns=rospy.get_namespace()
            )
            self.display_trajectory_publisher = rospy.Publisher(
                rospy.get_namespace() + "move_group/display_planned_path",
                moveit_msgs.msg.DisplayTrajectory,
                queue_size=20,
            )

            if self.is_gripper_present:
                gripper_group_name = "gripper"
                self.gripper_group = moveit_commander.MoveGroupCommander(
                    gripper_group_name, ns=rospy.get_namespace()
                )

            rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
        except Exception as e:
            print(e)
            self.is_init_success = False
        else:
            self.is_init_success = True

    def flush_data(self) -> None:
        self.data = []

    def add_data_point(self) -> None:
        data = self.data
        joint_vals = self.get_joint_values()
        pose = self.get_end_pose()
        data.append(np.round(joint_vals + pose, decimals=3))
        self.data = data

    def generate_random_joint_values(self) -> list:
        """generate a valid (within limits) position for the joints"""
        joint_limits = self.actuator_limits
        joint_1 = round(random.uniform(-joint_limits[0], joint_limits[0]), 3)
        joint_2 = round(random.uniform(-joint_limits[1], joint_limits[1]), 3)
        joint_3 = round(random.uniform(-joint_limits[2], joint_limits[2]), 3)
        joint_4 = round(random.uniform(-joint_limits[3], joint_limits[3]), 3)
        joint_5 = round(random.uniform(-joint_limits[4], joint_limits[4]), 3)
        joint_6 = round(random.uniform(-joint_limits[5], joint_limits[5]), 3)
        joint_7 = round(random.uniform(-joint_limits[6], joint_limits[6]), 3)

        return [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7]

    def explore_joint_motion(self, num_positions=2) -> bool:
        rospy.loginfo("Exploring joint motion in random positions...")
        dof = self.degrees_of_freedom
        success = True
        for n in range(num_positions):
            rospy.loginfo(f"Moving to random position {n} of {num_positions}...")
            rand_joint_vals = self.generate_random_joint_values()
            self.go_to_joint_target(rand_joint_vals)
            self.add_data_point()
            for i in range(dof):
                rospy.loginfo(f"Exploring positive displacements for joint {i}...")
                for _ in range(2):
                    self.shift_joint_value(self.joint_shift, i)
                    self.add_data_point()
                self.go_to_joint_target(rand_joint_vals)
                self.add_data_point()
                rospy.loginfo(f"Exploring negative displacements for joint {i}...")
                for _ in range(2):
                    self.shift_joint_value(-1 * self.joint_shift, i)
                    self.add_data_point()
                self.go_to_joint_target(rand_joint_vals)
                self.add_data_point()

        return success

    def go_to_joint_target(self, target: list):
        arm_group = self.arm_group
        arm_group.set_joint_value_target(target)
        success = arm_group.go(wait=True)
        return success

    def is_joint_target_valid(self, joint_target: list) -> bool:
        return True

    def is_pose_target_valid(self, pose_target: list) -> bool:
        return True

    def reach_named_position(self, target: str) -> bool:
        arm_group = self.arm_group
        # Going to one of those targets
        rospy.loginfo(f"Going to named target position: {target}")
        # Set the target
        arm_group.set_named_target(target)
        # Plan the trajectory
        (_, trajectory_message, _, _) = arm_group.plan()
        # (success_flag, trajectory_message, planning_time, error_code) =
        # arm_group.plan()
        # Execute the trajectory and block while it's not finished
        return arm_group.execute(trajectory_message, wait=True)

    def get_joint_values(self) -> list:
        arm_group = self.arm_group
        joint_positions = arm_group.get_current_joint_values()
        return joint_positions

    def get_end_position(self) -> list:
        arm_group = self.arm_group
        pose = arm_group.get_current_pose()
        pos = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
        return pos

    def get_end_rpy(self) -> list:
        arm_group = self.arm_group
        rpy = arm_group.get_current_rpy()
        return rpy

    def get_end_pose(self) -> list:
        pos = self.get_end_position()
        rpy = self.get_end_rpy()

        return pos + rpy

    def shift_joint_value(self, joint_shift: float, joint_idx: int) -> bool:
        if joint_idx > 7 or joint_idx < 0:
            return False
        arm_group = self.arm_group
        success = True
        # Get the current joint positions
        joint_positions = arm_group.get_current_joint_values()
        joint_positions[joint_idx] += joint_shift
        # Set the goal joint tolerance
        self.arm_group.set_goal_joint_tolerance(self.tolerance)
        arm_group.set_joint_value_target(joint_positions)
        # Plan and execute in one command
        success &= arm_group.go(wait=True)
        return success

    def shift_joint_values(self, joint_shifts: list) -> bool:
        arm_group = self.arm_group
        success = True
        # Get the current joint positions
        joint_positions = arm_group.get_current_joint_values()
        # Set the goal joint tolerance
        self.arm_group.set_goal_joint_tolerance(self.tolerance)
        print(joint_positions)
        target_values = np.add(joint_positions, joint_shifts)
        print(target_values)
        arm_group.set_joint_value_target(target_values)
        # Plan and execute in one command
        success &= arm_group.go(wait=True)
        return success

    def get_cartesian_pose(self) -> list:
        arm_group = self.arm_group

        # Get the current pose and display it
        pose = arm_group.get_current_pose()
        rospy.loginfo("Actual cartesian pose is : ")
        rospy.loginfo(pose.pose)

        return pose.pose

    def reach_gripper_position(self, relative_position) -> bool:
        # gripper_group = self.gripper_group
        # We only have to move this joint because all others are mimic!
        gripper_joint = self.robot.get_joint(self.gripper_joint_name)
        gripper_max_absolute_pos = gripper_joint.max_bound()
        gripper_min_absolute_pos = gripper_joint.min_bound()
        try:
            val = gripper_joint.move(
                relative_position
                * (gripper_max_absolute_pos - gripper_min_absolute_pos)
                + gripper_min_absolute_pos,
                True,
            )
            return val
        except Exception:
            return False

    def convert_joint_values(self, joint_values):
        return

    def save_csv(self, filename="test.csv") -> None:
        data = self.data

        headings = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
            "joint_7",
            "pos_x",
            "pos_y",
            "pos_z",
            "rot_r",
            "rot_p",
            "rot_y",
        ]

        if data == []:
            rospy.logerr("No data found, writing to csv failed...")
            return None
        df = pd.DataFrame(data)
        df.to_csv(filename, header=headings)


def main():
    kortex = StateCollector()

    if not kortex.is_init_success:
        return

    kortex.reach_named_position("home")
    kortex.explore_joint_motion()
    kortex.reach_named_position("home")
    kortex.save_csv()


if __name__ == "__main__":
    main()
