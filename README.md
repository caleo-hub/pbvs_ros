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
$ cd ~/catkin_ws/src/Universal_Robots_ROS_Driver

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

### Executando o experimento


Escolha uma das opções 
```bash
# Ambiente com RVIZ, Gazebo e Camera 
roslaunch pbvs_ros sim.launch

# Ambiente apenas Gazebo e Camera 
roslaunch pbvs_ros simple_sim.launch
```

Quando o robô estiver posicionado, execute o controlador em outro terminal
```bash
roslaunch pbvs_ros controlador.py
```