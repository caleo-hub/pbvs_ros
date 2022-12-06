#!/usr/bin/python3
from math import pi, tau, dist, fabs, cos
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import copy

import numpy as np
import rospy
import sys
import moveit_commander
import moveit_msgs.msg
from std_msgs.msg import Float32MultiArray


class Controller:
  def __init__(self) -> None:
    moveit_commander.roscpp_initialize(sys.argv)
    self.robot = moveit_commander.RobotCommander()
    self.scene = moveit_commander.PlanningSceneInterface()
    self.group_name = "manipulator"
    self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
    self.display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20,
    )
    self.planning_frame = self.move_group.get_planning_frame()
    self.eef_link = self.move_group.get_end_effector_link()
    self.group_names = self.robot.get_group_names()
    
    self.pose_sub = rospy.Subscriber("/pose_error", Float32MultiArray, self.callback,queue_size=1, buff_size=2**24)
  
  
  def callback(self,msg):
      self.dx, self.dy, self.yaw = msg.data[0], msg.data[1], msg.data[2]
      yaw_done = self.control_loop_yaw(self.yaw)
      
      if yaw_done:
          pass
          
      
      
    
  def control_loop_yaw(self,yaw):
    if abs(yaw)>0.05:
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = tau/4
        joint_goal[1] = -0.1055*tau
        joint_goal[2] = tau/24
        joint_goal[3] = -0.1833*tau
        joint_goal[4] = -tau/4

        k = 0.1
        erro = 0 - yaw
        joint_goal[5] = joint_goal[5] - k*erro
        self.move_group.go(joint_goal)
        return False
    else:
        self.move_group.stop()
        return True
    
  def control_loop_pos(self):
      pass
        
    
      
          
      
    
    

  def wrap_angle(self, angle):
    angle = (angle + np.pi) % (2 * np.pi) - np.pi
    return angle

def main():
  a = Controller()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    rospy.loginfo("Shutting down")
  
if __name__ == '__main__':
    rospy.init_node('controlador', anonymous=False)
    main()