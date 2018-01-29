#!/usr/bin/env python
# -*- coding: utf-8 -*-
#Created on Mon Jan 22 13:36:32 2018
#@author: Bas Janssen

import sys
import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
import tf
import math
from descartes_planning_service.srv import generate_motion_plan
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import actionlib

global scene

def execute_trajectory(path):
  #Creates the SimpleActionClient, passing the type of the action
  client = actionlib.SimpleActionClient('joint_trajectory_action', FollowJointTrajectoryAction)

  #Waits until the action server has started up and started listening for goals.
  client.wait_for_server()

  #Creates a goal to send to the action server.
  goal = FollowJointTrajectoryGoal()
  goal.trajectory = path
  goal.goal_time_tolerance = rospy.Duration(0.01)
  #Sends the goal to the action server.
  client.send_goal(goal)
  #Waits for the server to finish performing the action.
  client.wait_for_result()

  #Returns out the result of executing the action
  return client.get_result()

def onshutdownhook():
  global scene
  scene.remove_attached_object("ee_link")
  scene.remove_world_object("box1")
  scene.remove_world_object("box2")
  scene.remove_world_object("box3")

def main(argv):
  rospy.init_node("PickPlaceDemo")
  
  rospy.on_shutdown(onshutdownhook)  
  
  rospy.wait_for_service("descartes_generate_motion_plan")
  print "Planning service availabe" 
  
  global scene
  scene = PlanningSceneInterface()
  robot = RobotCommander()
  
  path = JointTrajectory()
  
  rospy.sleep(2)
  
  discretization = math.pi/30.0
  
  #scene.is_diff = True
  
  quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)  
  
  p = PoseStamped()
  p.header.frame_id = robot.get_planning_frame()
  p.pose.position.x = 0.700
  p.pose.position.y = 0.000
  p.pose.position.z = 0.060
  p.pose.orientation.x = quaternion[0]
  p.pose.orientation.y = quaternion[1]
  p.pose.orientation.z = quaternion[2]
  p.pose.orientation.w = quaternion[3]
  scene.add_box("box1", p, (0.15, 0.15, 0.15))
  
  p.pose.position.x = 0.700
  p.pose.position.y = 0.250
  p.pose.position.z = 0.060
  scene.add_box("box2", p, (0.15, 0.15, 0.15))
  
  p.pose.position.x = 0.700
  p.pose.position.y = 0.500
  p.pose.position.z = 0.060
  scene.add_box("box3", p, (0.15, 0.15, 0.15))
  
  rospy.sleep(2)  
  
  del quaternion
  
  #Pick the first box
  quaternion = tf.transformations.quaternion_from_euler(0.0, (math.pi / 2.0), (math.pi / 2.0))
  
  poses = PoseArray()
 
  pose_target = Pose()
  pose_target.position.x = 0.700
  pose_target.position.y = 0.000
  pose_target.position.z = 0.155
  pose_target.orientation.x = quaternion[0]
  pose_target.orientation.y = quaternion[1]
  pose_target.orientation.z = quaternion[2]
  pose_target.orientation.w = quaternion[3]
  
  poses.poses.append(pose_target)
  
  try:
      planningrequest = rospy.ServiceProxy('descartes_generate_motion_plan', generate_motion_plan)
      plan = planningrequest(poses, "world", "ee_link", rospy.get_param("/controller_joint_names"), 5, "Y_AXIS", discretization, robot.get_current_state().joint_state)
      path = plan.plan
      print "Plan generated"
  except rospy.ServiceException, e:
      print "Service call failed: %s"%e
  
  result = execute_trajectory(path)
  if result.error_code != 0:
    print "Error occured during path planning. Not continuing."
    quit()
  
  rospy.sleep(1)
  
  scene.attach_box("ee_link", "box1")

  rospy.sleep(1)
  
  #Place the first box
  
  del poses
  
  poses = PoseArray()
  
  del pose_target
  
  pose_target = Pose()
  
  pose_target.position.x = 0.700
  pose_target.position.y = 0.000
  pose_target.position.z = 0.315
  pose_target.orientation.x = quaternion[0]
  pose_target.orientation.y = quaternion[1]
  pose_target.orientation.z = quaternion[2]
  pose_target.orientation.w = quaternion[3]  
  
  poses.poses.append(pose_target)

  del pose_target
  
  pose_target = Pose() 
  
  pose_target.position.x = 0.000
  pose_target.position.y = 0.600
  pose_target.position.z = 0.315
  pose_target.orientation.x = quaternion[0]
  pose_target.orientation.y = quaternion[1]
  pose_target.orientation.z = quaternion[2]
  pose_target.orientation.w = quaternion[3]  
  
  poses.poses.append(pose_target)  

  del pose_target
 
  pose_target = Pose()
  
  pose_target.position.x = 0.000
  pose_target.position.y = 0.600
  pose_target.position.z = 0.155
  pose_target.orientation.x = quaternion[0]
  pose_target.orientation.y = quaternion[1]
  pose_target.orientation.z = quaternion[2]
  pose_target.orientation.w = quaternion[3]  
  
  poses.poses.append(pose_target)
  
  try:
      planningrequest = rospy.ServiceProxy('descartes_generate_motion_plan', generate_motion_plan)
      plan = planningrequest(poses, "world", "ee_link", rospy.get_param("/controller_joint_names"), 5, "Y_AXIS", discretization, robot.get_current_state().joint_state)
      path = plan.plan
      print "Plan generated"
  except rospy.ServiceException, e:
      print "Service call failed: %s"%e
  
  execute_trajectory(path)
  if result.error_code != 0:
    print "Error occured during path planning. Not continuing."
    quit()
    
  rospy.sleep(1)
  
  scene.remove_attached_object("ee_link")
  
  rospy.sleep(2)  
  
  #Pick the second box
  del poses
  
  poses = PoseArray()

  del pose_target
  
  pose_target = Pose()
  
  pose_target.position.x = 0.000
  pose_target.position.y = 0.600
  pose_target.position.z = 0.315
  pose_target.orientation.x = quaternion[0]
  pose_target.orientation.y = quaternion[1]
  pose_target.orientation.z = quaternion[2]
  pose_target.orientation.w = quaternion[3]
  poses.poses.append(pose_target)

  del pose_target
  
  pose_target = Pose()
  
  pose_target.position.x = 0.700
  pose_target.position.y = 0.250
  pose_target.position.z = 0.300
  pose_target.orientation.x = quaternion[0]
  pose_target.orientation.y = quaternion[1]
  pose_target.orientation.z = quaternion[2]
  pose_target.orientation.w = quaternion[3]
  poses.poses.append(pose_target)

  del pose_target
  
  pose_target = Pose()
  
  pose_target.position.x = 0.700
  pose_target.position.y = 0.250
  pose_target.position.z = 0.155
  pose_target.orientation.x = quaternion[0]
  pose_target.orientation.y = quaternion[1]
  pose_target.orientation.z = quaternion[2]
  pose_target.orientation.w = quaternion[3]
  poses.poses.append(pose_target)
  
  try:
      planningrequest = rospy.ServiceProxy('descartes_generate_motion_plan', generate_motion_plan)
      plan = planningrequest(poses, "world", "ee_link", rospy.get_param("/controller_joint_names"), 5, "Y_AXIS", discretization, robot.get_current_state().joint_state)
      path = plan.plan
      print "Plan generated"
  except rospy.ServiceException, e:
      print "Service call failed: %s"%e
  
  execute_trajectory(path)   
  if result.error_code != 0:
    print "Error occured during path planning. Not continuing."
    quit()
    
  rospy.sleep(1)
  
  scene.attach_box("ee_link", "box2")
  
  rospy.sleep(1)
  
  #Place the second box
  
  del poses
  
  poses = PoseArray()

  del pose_target
  
  pose_target = Pose()
  
  pose_target.position.x = 0.700
  pose_target.position.y = 0.250
  pose_target.position.z = 0.315
  pose_target.orientation.x = quaternion[0]
  pose_target.orientation.y = quaternion[1]
  pose_target.orientation.z = quaternion[2]
  pose_target.orientation.w = quaternion[3]
  poses.poses.append(pose_target)

  del pose_target
  
  pose_target = Pose()
  
  pose_target.position.x = 0.000
  pose_target.position.y = 0.600
  pose_target.position.z = 0.315
  pose_target.orientation.x = quaternion[0]
  pose_target.orientation.y = quaternion[1]
  pose_target.orientation.z = quaternion[2]
  pose_target.orientation.w = quaternion[3]
  poses.poses.append(pose_target)

  del pose_target
  
  pose_target = Pose()
 
  pose_target.position.x = 0.000
  pose_target.position.y = 0.600
  pose_target.position.z = 0.306
  pose_target.orientation.x = quaternion[0]
  pose_target.orientation.y = quaternion[1]
  pose_target.orientation.z = quaternion[2]
  pose_target.orientation.w = quaternion[3]
  poses.poses.append(pose_target)
  
  try:
      planningrequest = rospy.ServiceProxy('descartes_generate_motion_plan', generate_motion_plan)
      plan = planningrequest(poses, "world", "ee_link", rospy.get_param("/controller_joint_names"), 5, "Y_AXIS", discretization, robot.get_current_state().joint_state)
      path = plan.plan
      print "Plan generated"
  except rospy.ServiceException, e:
      print "Service call failed: %s"%e
  
  execute_trajectory(path) 
  if result.error_code != 0:
    print "Error occured during path planning. Not continuing."
    quit()
    
  rospy.sleep(1)
  
  scene.remove_attached_object("ee_link")
  
  rospy.sleep(1)
  
  del poses
  
  poses = PoseArray()
 
  del pose_target
  
  pose_target = Pose()
  
  pose_target.position.x = 0.000
  pose_target.position.y = 0.600
  pose_target.position.z = 0.315
  pose_target.orientation.x = quaternion[0]
  pose_target.orientation.y = quaternion[1]
  pose_target.orientation.z = quaternion[2]
  pose_target.orientation.w = quaternion[3]
  poses.poses.append(pose_target)
  
  del pose_target
  
  pose_target = Pose()
  
  pose_target.position.x = 0.700
  pose_target.position.y = 0.500
  pose_target.position.z = 0.155
  pose_target.orientation.x = quaternion[0]
  pose_target.orientation.y = quaternion[1]
  pose_target.orientation.z = quaternion[2]
  pose_target.orientation.w = quaternion[3]
  poses.poses.append(pose_target)
  
  try:
      planningrequest = rospy.ServiceProxy('descartes_generate_motion_plan', generate_motion_plan)
      plan = planningrequest(poses, "world", "ee_link", rospy.get_param("/controller_joint_names"), 5, "Y_AXIS", discretization, robot.get_current_state().joint_state)
      path = plan.plan
      print "Plan generated"
  except rospy.ServiceException, e:
      print "Service call failed: %s"%e

  execute_trajectory(path)
  if result.error_code != 0:
    print "Error occured during path planning. Not continuing."
    quit()
    
  rospy.sleep(1)
  
  scene.attach_box("ee_link", "box3")
  
  rospy.sleep(1)
  
  del poses
  
  poses = PoseArray()
 
  del pose_target
  
  pose_target = Pose()
  
  pose_target.position.x = 0.700
  pose_target.position.y = 0.400
  pose_target.position.z = 0.400
  pose_target.orientation.x = quaternion[0]
  pose_target.orientation.y = quaternion[1]
  pose_target.orientation.z = quaternion[2]
  pose_target.orientation.w = quaternion[3]
  poses.poses.append(pose_target)
  
  del pose_target
  
  pose_target = Pose()
  
  pose_target.position.x = 0.000
  pose_target.position.y = 0.600
  pose_target.position.z = 0.457
  pose_target.orientation.x = quaternion[0]
  pose_target.orientation.y = quaternion[1]
  pose_target.orientation.z = quaternion[2]
  pose_target.orientation.w = quaternion[3]
  poses.poses.append(pose_target)
  del pose_target
  
  pose_target = Pose()
  pose_target.position.x = 0.000
  pose_target.position.y = 0.600
  pose_target.position.z = 0.457
  pose_target.orientation.x = quaternion[0]
  pose_target.orientation.y = quaternion[1]
  pose_target.orientation.z = quaternion[2]
  pose_target.orientation.w = quaternion[3]
  poses.poses.append(pose_target)
  
  try:
      planningrequest = rospy.ServiceProxy('descartes_generate_motion_plan', generate_motion_plan)
      plan = planningrequest(poses, "world", "ee_link", rospy.get_param("/controller_joint_names"), 5, "Y_AXIS", discretization, robot.get_current_state().joint_state)
      path = plan.plan
      print "Plan generated"
  except rospy.ServiceException, e:
      print "Service call failed: %s"%e
  
  execute_trajectory(path)
  if result.error_code != 0:
    print "Error occured during path planning. Not continuing."
    quit()
    
  rospy.sleep(1)
  
  scene.remove_attached_object("ee_link")
  
  rospy.sleep(1)
  
  while not rospy.is_shutdown():
    pass

if __name__ == "__main__":
  try:
    main(sys.argv)
  except rospy.ROSInteruptException:
    pass