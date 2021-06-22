# Creating a map using Turtlebot3 and SLAM
This is documentation of how to set up your PC to control TurtleBot 3 Burger robot. And simulating it using Gazebo and SLAM to create a map to the robot's model environment in virtual world.


*Note: These instructions have been done on `Ubuntu 18.04.5 LTS bionic` and `ROS melodic` .* 

## Outline
- PC Set up
- Simulation with Gazebo
- creating the Map with SLAM

## 1-PC Set up
After making sure of the version of ubuntu and ROS using the commands `lsb_release -a` and `printenv ROS_DISTRO` on the terminal. Then install all the dependency packages required for turrlebot3 package.


```bash
$ sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy \
  ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc \
  ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan \
  ros-melodic-rosserial-arduino ros-melodic-rosserial-python \
  ros-melodic-rosserial-server ros-melodic-rosserial-client \
  ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server \
  ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro \
  ros-melodic-compressed-image-transport ros-melodic-rqt* \
  ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers
  ```
  
 you must have a source folder under the cankit workspace folder. Then download and install turtleBot3 packages.
 
 
 ```bash
$ sudo apt-get remove ros-melodic-dynamixel-sdk
$ sudo apt-get remove ros-melodic-turtlebot3-msgs
$ sudo apt-get remove ros-melodic-turtlebot3
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src/
$ git clone -b melodic-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
$ git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
 ```
 
 ((((HERE an image))))
 
 
 now build the downloaded packages, After that the set up will be completed.
  
  
  ```bash
  $ cd ~/catkin_ws && catkin_make
  
  ```
  (((HERE an Image)))
  
  ## 2-Simulation with Gazebo
  
  ### Gazebo Simulation
  *Note : `gazebo -vertion 9.0.0` is used.*
  
  
  start by installing the simulation package and build the package.
  
  
  ```bash
$ cd ~/catkin_ws/src/
$ git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ~/catkin_ws && catkin_make
  ```
  
  (((HERE an image)))
  
  
  To launch the simulation world in empty environment use these commands
  
  
  ```bash
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
  ```
  
  (((HERE an image)))
  
  in TurtleBot3 world environment
  
  ```bash
  $ export TURTLEBOT3_MODEL=burger
  $ roslaunch turtlebot3_gazebo turtlebot3_world.launch
   ```
   
   (((here image)))
   
   
   ### Operate TurtleBot3 Burger in Gazebo
   
   
   in a new terminal after running gazebo in the previous part, run the following command to teleoperate the turtleBot3.
   
   ```bash
    $ export TURTLEBOT3_MODEL=burger
   $ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
  ```
  
  (((here photo)))
  *Note: now you can control turtleBot3 burger using keyboard keys As shown above.*
  
  
  
  ## 3-creating the Map with SLAM
  
  first gazebo must be running using the previous command. and in new terminal run turtleBot3 SLAM.
  
  
 ```bash
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
  ```
  
  (((here photo)))
  
  
  create new terminal and run the teleoperation command in part 2.
  (((here photo)))
  
  
  now you can move turtleBot3 burger with keyboard to create map 
  ((here image))
  
  
  finally create new terminal ans save the map.
  
  ```bash 
  $ rosrun map_server map_saver -f ~/map
  ````
  
  (((here an image )))
  
  
  This map will help us navigate the robot and move from a location to destination in a given environment.
  
  
  
  





  
