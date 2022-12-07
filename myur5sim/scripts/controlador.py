#!/usr/bin/python3
from math import pi, tau, dist, cos, sin, sqrt
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
    self.pos_not_done = True
    self.pose_sub = rospy.Subscriber("/pose_error", Float32MultiArray, self.callback,queue_size=1, buff_size=2**24)
    
  
  def callback(self,msg):
      self.dx, self.dy, self.yaw = msg.data[0], msg.data[1], msg.data[2]
      yaw_done = self.control_loop_yaw(self.yaw)
      
      if yaw_done and self.pos_not_done:
          self.control_loop_pos(self.dx, self.dy)
          
          
      
      
    
  def control_loop_yaw(self,yaw):
    joint_initial = self.move_group.get_current_joint_values()
    
    if abs(yaw)>0.05:
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = tau/4
        joint_goal[1] = -tau/4
        joint_goal[2] = tau/4
        joint_goal[3] = -tau/4
        joint_goal[4] = -tau/4

        k = 0.1
        erro = 0 - yaw
        joint_goal[5] = joint_goal[5] - k*erro
        self.move_group.go(joint_goal)
        return False
    else:
        self.move_group.stop()
        return True
  def set_real_d(self, dx, dy):
    dx_frame = 0
    dy_frame = 0
    joint_goal = self.move_group.get_current_joint_values()
    neutral_angle = -tau/4
    joint_angle = joint_goal[5]
    angle_change = neutral_angle - joint_angle
    # Para eixo Y
    dx_frame += sin(angle_change)*dy
    dy_frame += cos(angle_change)*dy
    
    # Para eixo x
    dx_frame += cos(angle_change)*dx
    dy_frame += sin(angle_change)*dx
    
    dx_frame = round(dx_frame,4)
    dy_frame = round(dy_frame,4)
    return dx_frame, dy_frame
    
  def control_loop_pos(self,dx, dy):
    if dist([dx],[dy])>=0.5:
      # self.move_group.set_planning_time(10)
      #self.move_group.set_goal_tolerance(0.1)
      waypoints = []
      scale = 1/10
      k = 0.5
      wpose = self.move_group.get_current_pose().pose
      dx_frame, dy_frame = self.set_real_d(dx,dy)
      print(dx_frame, dy_frame)
      wpose.position.x += k*scale * dx_frame
      wpose.position.y += k*scale * dy_frame  
      
      # SET POSE ===========================
      # self.move_group.set_pose_target(wpose)
      # success = self.move_group.go(wait=True)
      # self.move_group.stop()
      # self.move_group.clear_pose_targets()
      
      #=====================================
      
      # CARTESIANO ========================== 
      waypoints.append(copy.deepcopy(wpose))
      (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.001, 1)
      print('fraction',fraction)
      self.move_group.execute(plan, wait=True)
      self.move_group.stop()
      #========================================
    else:
      self.pos_not_done = False
      print('pronto')
    
  def display_trajectory(self,plan):
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = self.robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    self.display_trajectory_publisher.publish(display_trajectory)
      
        
        
    
      
          
      
    
    

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