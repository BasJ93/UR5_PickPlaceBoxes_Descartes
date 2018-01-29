#!/usr/bin/env python
import sys
import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
import tf

def main(argv):
  rospy.init_node("Table_mesh_inserter")
  
  scene = PlanningSceneInterface()
  robot = RobotCommander()
  
  while(scene._pub_co.get_num_connections() < 1):
      pass
  
  quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)  
  
  p = PoseStamped()
  p.header.frame_id = robot.get_planning_frame()
  p.pose.position.x = 0.3250
  p.pose.position.y = 0.4250
  p.pose.position.z = -0.540
  p.pose.orientation.x = quaternion[0]
  p.pose.orientation.y = quaternion[1]
  p.pose.orientation.z = quaternion[2]
  p.pose.orientation.w = quaternion[3]
  scene.add_box("table", p, (1.2, 1.2, 1.05))
  
  p.pose.position.x = 0.0
  p.pose.position.y = 0.0
  p.pose.position.z = -0.0075
  scene.add_box("plate", p, (0.25, 0.25, 0.015))
  
  while not rospy.is_shutdown():
    pass

if __name__ == "__main__":
  try:
    main(sys.argv)
  except rospy.ROSInteruptException:
    pass
