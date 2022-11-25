# PBVS_ROS

## Instalando o pacote do universal-robot (ur5)

```bash
# source global ros
$ source /opt/ros/$ROS_DISTRO/setup.bash

# change directory
$ cd ~/catkin_ws

# clone the driver
$ git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver

# change directory
$ cd ~/catkin_ws/src

# clone the description. Currently, it is necessary to use the melodic-devel branch.
$ git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git src/universal_robot

# install dependencies
$ sudo apt update -qq
$ rosdep update
$ rosdep install --from-paths src --ignore-src -y

# change directory
$ cd ~/catkin_ws

# build the workspace
$ catkin_make

# activate the workspace (ie: source it)
$ source devel/setup.bash
```