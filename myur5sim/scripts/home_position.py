#!/usr/bin/python3
import sys
import rospy
import moveit_commander
import moveit_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)

planning_frame = move_group.get_planning_frame()

eef_link = move_group.get_end_effector_link()

group_names = robot.get_group_names()


print(robot.get_current_state())

planning_frame = move_group.get_planning_frame()

eef_link = move_group.get_end_effector_link()

group_names = robot.get_group_names()


print(robot.get_current_state())


print("============ Moving to Home Position")
# We get the joint values from the group and change some of the values:
joint_goal = move_group.get_current_joint_values()
joint_goal[0] = tau/4
joint_goal[1] = -tau/4
joint_goal[2] = tau/4
joint_goal[3] = -tau/4
joint_goal[4] = -tau/4
joint_goal[5] = -tau/4

move_group.go(joint_goal, wait=True)
move_group.stop()

# We get the joint values from the group and change some of the values:
joint_goal = move_group.get_current_joint_values()
joint_goal[0] = tau/4
joint_goal[1] = -0.1055*tau
joint_goal[2] = tau/24
joint_goal[3] = -0.1833*tau
joint_goal[4] = -tau/4
joint_goal[5] = -tau/4

move_group.go(joint_goal, wait=True)
move_group.stop()
