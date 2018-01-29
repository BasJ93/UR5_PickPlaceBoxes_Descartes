#!/usr/bin/env python
# -*- coding: utf-8 -*-
#Created on Mon Jan 22 13:36:32 2018
#@author: bas

#First create the boxes to stack. Move the robot to the first box to pick. Add the box to the robot with attache_box(link, name). Move the arm to the place point. Release the box with remove_attached_object(link, name). Repeat.

import sys
import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from geometry_msgs.msg import PoseStamped, Pose
import tf
import math

global scene

def onshutdownhook():
  global scene
  #scene.remove_attached_object("tool0", "attached_box")
  scene.remove_world_object("box1")
  scene.remove_world_object("box2")
  scene.remove_world_object("box3")

def main(argv):
  #moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node("PickPlaceDemo")
  
  rospy.on_shutdown(onshutdownhook)  
  
  transform = tf.TransformListener()  
  
  global scene
  scene = PlanningSceneInterface()
  robot = RobotCommander()
  group = MoveGroupCommander("manipulator")
  
  rospy.sleep(2)
  
  #scene.is_diff = True
  
  #t = transform.getLatestCommonTime("/tool0", "/world")
  #tool0_pose, tool0_quaternion = transform.lookupTransform("world", "tool0", t)
  
  quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)  
  
  p = PoseStamped()
  p.header.frame_id = robot.get_planning_frame()
  p.pose.position.x = -0.700
  p.pose.position.y = 0.000
  p.pose.position.z = 0.060
  p.pose.orientation.x = quaternion[0]
  p.pose.orientation.y = quaternion[1]
  p.pose.orientation.z = quaternion[2]
  p.pose.orientation.w = quaternion[3]
  scene.add_box("box1", p, (0.15, 0.15, 0.15))
  
  p.pose.position.x = -0.700
  p.pose.position.y = -0.250
  p.pose.position.z = 0.060
  scene.add_box("box2", p, (0.15, 0.15, 0.15))
  
  p.pose.position.x = -0.700
  p.pose.position.y = -0.500
  p.pose.position.z = 0.060
  scene.add_box("box3", p, (0.15, 0.15, 0.15))
  
  rospy.sleep(5)  
  
  quaternion = tf.transformations.quaternion_from_euler(0.0, (math.pi / 2.0), 0.0)    
  
  pose_target = Pose()
  pose_target.position.x = -0.700
  pose_target.position.y = 0.000
  pose_target.position.z = 0.155
  pose_target.orientation.x = quaternion[0]
  pose_target.orientation.y = quaternion[1]
  pose_target.orientation.z = quaternion[2]
  pose_target.orientation.w = quaternion[3]
  
  group.set_pose_target(pose_target)
  
  group.go(wait=True)
  
  rospy.sleep(1)  
  
  scene.attach_box("tool0", "box1")

  rospy.sleep(1)
  
  pose_target.position.x = 0.000
  pose_target.position.y = -0.700
  pose_target.position.z = 0.155
  
  group.set_pose_target(pose_target)
  
  group.go(wait=True)
  
  rospy.sleep(1)  
  
  scene.remove_attached_object("tool0")
  
  rospy.sleep(1)  
  
  pose_target.position.x = -0.700
  pose_target.position.y = -0.250
  pose_target.position.z = 0.155
  
  group.set_pose_target(pose_target)
  
  group.go(wait=True)
  
  rospy.sleep(1)
  
  scene.attach_box("tool0", "box2")
  
  rospy.sleep(1)
  
  pose_target.position.x = 0.000
  pose_target.position.y = -0.700
  pose_target.position.z = 0.306
  
  group.set_pose_target(pose_target)
  
  group.go(wait=True)
  
  rospy.sleep(1)
  
  scene.remove_attached_object("tool0")
  
  rospy.sleep(1)  
  
  pose_target.position.x = -0.700
  pose_target.position.y = -0.500
  pose_target.position.z = 0.155
  
  group.set_pose_target(pose_target)
  
  group.go(wait=True)
  
  rospy.sleep(1)
  
  scene.attach_box("tool0", "box3")
  
  rospy.sleep(1)
  
  pose_target.position.x = 0.000
  pose_target.position.y = -0.700
  pose_target.position.z = 0.457
  
  group.set_pose_target(pose_target)
  
  group.go(wait=True)
  
  rospy.sleep(1)
  
  scene.remove_attached_object("tool0")
  
  while not rospy.is_shutdown():
    pass

if __name__ == "__main__":
  try:
    main(sys.argv)
  except rospy.ROSInteruptException:
    pass