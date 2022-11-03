#!/usr/bin/env python3

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import e, pi
from std_srvs.srv import Empty

import numpy as np
import pandas as pd
# import csv

class StateCollector(object):
  """StateCollector"""
  def __init__(self):

    # Initialize the node
    super(StateCollector, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('state_collector')

    try:
      self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
      if self.is_gripper_present:
        gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
        self.gripper_joint_name = gripper_joint_names[0]
      else:
        self.gripper_joint_name = ""
      self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

      # Create the MoveItInterface necessary objects
      arm_group_name = "arm"
      self.robot = moveit_commander.RobotCommander("robot_description")
      self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
      self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
      self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

      if self.is_gripper_present:
        gripper_group_name = "gripper"
        self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

      rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
    except Exception as e:
      print (e)
      self.is_init_success = False
    else:
      self.is_init_success = True


  def reach_named_position(self, target):
    arm_group = self.arm_group
    # Going to one of those targets
    rospy.loginfo("Going to named target " + target)
    # Set the target
    arm_group.set_named_target(target)
    # Plan the trajectory
    (success_flag, trajectory_message, planning_time, error_code) = arm_group.plan()
    # Execute the trajectory and block while it's not finished
    return arm_group.execute(trajectory_message, wait=True)

  def get_joint_values(self):
    arm_group = self.arm_group
    joint_positions = arm_group.get_current_joint_values()
    return joint_positions

  def get_end_position(self):
    arm_group = self.arm_group
    pose = arm_group.get_current_pose()
    pos = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
    return pos

  def get_end_rpy(self):
    arm_group = self.arm_group
    rpy = arm_group.get_current_rpy()
    return rpy

  def get_end_pose(self):
      pos = self.get_end_position()
      rpy = self.get_end_rpy()

      return pos + rpy 

  def shift_joint_values(self, joint_shifts, tolerance):
    arm_group = self.arm_group
    success = True
    # Get the current joint positions
    joint_positions = arm_group.get_current_joint_values()
    # Set the goal joint tolerance
    self.arm_group.set_goal_joint_tolerance(tolerance)
    target_values = np.add(joint_positions, joint_shifts)
    arm_group.set_joint_value_target(target_values)
    # Plan and execute in one command
    success &= arm_group.go(wait=True)
    return success

  def get_cartesian_pose(self):
    arm_group = self.arm_group

    # Get the current pose and display it
    pose = arm_group.get_current_pose()
    rospy.loginfo("Actual cartesian pose is : ")
    rospy.loginfo(pose.pose)

    return pose.pose

  def reach_gripper_position(self, relative_position):
    gripper_group = self.gripper_group
    # We only have to move this joint because all others are mimic!
    gripper_joint = self.robot.get_joint(self.gripper_joint_name)
    gripper_max_absolute_pos = gripper_joint.max_bound()
    gripper_min_absolute_pos = gripper_joint.min_bound()
    try:
      val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
      return val
    except:
      return False 

  def convert_joint_values(self, joint_values):
    return 

def main():
  kortex = StateCollector()

  if not kortex.is_init_success:
      return 

  kortex.reach_named_position("vertical")

  data = []

  joint_shift = [0, 0.1, 0, 0, 0, 0, 0]
  for x in range(24):
    rospy.loginfo("Step " + str(x + 1) + " of 100")
    kortex.shift_joint_values(joint_shift, tolerance=0.01)
    joints = kortex.get_joint_values()
    pose = kortex.get_end_pose()
    data.append(joints + pose)


  data = np.round(data, decimals=2)

  headings = [
          "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7", 
          # "shift_1", "shift_2", "shift_3", "shift_4", "shift_5", "shift_6", "shift_7", 
          "pos_x", "pos_y", "pos_z", "rot_r", "rot_p", "rot_y"
          ]
  
  df = pd.DataFrame(data)
  df.to_csv("test.csv", header=headings)

if __name__ == '__main__':
  main()

