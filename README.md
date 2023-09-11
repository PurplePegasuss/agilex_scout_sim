[TOC]

# Scout Simulation Operation Process

### SLAM in Gazebo and Rviz
<p align = "center">
  <img src = "animations/Gazebo_sim_edited.gif" height = "240px" style="margin:10px 10px">
</p>

## 1.	Introduction of Function Package

```
├── scout_control
├── scout_description
└── scout_gazebo_sim
```

​	scout_gazebo_sim：The folder is gazebo simulation function package

​	scout_control: The folder is simulation controller function package

​	scout_description: The folder is the function package of model file

## 2.	Environment

### Development Environment

​	ubuntu 20.04 + [ROS Noetic desktop full](http://wiki.ros.org/noetic/Installation/Ubuntu)

### Download and install required function package

​	Download and install packages provided by ROS:

```
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-gazebo-ros ros-noetic-gazebo-ros-control ros-noetic-joint-state-publisher-gui ros-noetic-teleop-twist-keyboard ros-noetic-velodyne-description ros-noetic-velodyne-simulator ros-noetic-gmapping ros-noetic-pointcloud-to-laserscan ros-noetic-teb-local-planner ros-noetic-dwa-local-planner ros-noetic-move-base
```

## 3.	About Usage
If you already have ROS workspace configured, go to the src folder and clone agilex_scout_sim repository content:
```
cd catkin_ws/src
git clone https://github.com/PurplePegasuss/agilex_scout_sim.git
```
Return to the catkin_ws folder and build the prepared packages:
```
cd ..
source /opt/ros/noetic/setup.bash
catkin_make
```
Source build files:
```
source devel/setup.sh
```
Launch simulation:
```
roslaunch scout_gazebo_sim scout_mini_playpen.launch
```
