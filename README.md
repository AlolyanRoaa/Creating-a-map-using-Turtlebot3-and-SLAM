# Creating a map using Turtlebot3 and SLAM
This is documentation of how to set up your PC to control TurtleBot 3 Burger robot. And simulating it using Gazebo and SLAM to create a map to the robot's model environment in virtual world.


*Note: These instructions have been done on `Ubuntu 18.04.5 LTS bionic` and `ROS melodic` .* 

## Outline
- PC Set up
- Simulation with Gazebo
- creating the Map with SLAM
- Purpose of The Map

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
  
  ![01-install dependency package](https://github.com/AlolyanRoaa/Creating-a-map-using-Turtlebot3-and-SLAM/blob/main/01-install%20dependency%20package.PNG)
  
  
  
 to install turtleBot3 package
  
  
  ```bash
$ sudo apt-get remove ros-melodic-dynamixel-sdk
$ sudo apt-get remove ros-melodic-turtlebot3-msgs
$ sudo apt-get remove ros-melodic-turtlebot3
``` 
 
![02-install turtleBot3 packag](https://github.com/AlolyanRoaa/Creating-a-map-using-Turtlebot3-and-SLAM/blob/main/02-install%20turtleBot3%20packag.PNG) 
 
 
 you must have a source folder under the cankit workspace folder. Then download and install turtleBot3 packages.
 
 
 ```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src/
$ git clone -b melodic-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
$ git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
 ```
 
 ![03-download turtleBot3 packages](https://github.com/AlolyanRoaa/Creating-a-map-using-Turtlebot3-and-SLAM/blob/main/03-download%20turtleBot3%20packages.PNG)
 
 
 now build the downloaded packages, After that the set up will be completed.
  
  
  ```bash
  $ cd ~/catkin_ws && catkin_make
  
  ```
  
  ![04-build the downloaded package](https://github.com/AlolyanRoaa/Creating-a-map-using-Turtlebot3-and-SLAM/blob/main/04-build%20the%20downloaded%20package.PNG)
  
  
  ## 2-Simulation with Gazebo
  
  ### Gazebo Simulation
  *Note : `gazebo -vertion 9.0.0` is used.*
  
  
  start by installing the simulation package and build the package.
  
  
  ```bash
$ cd ~/catkin_ws/src/
$ git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ~/catkin_ws && catkin_make
  ```
  
  ![05-installing the simulation packag.PNG](https://github.com/AlolyanRoaa/Creating-a-map-using-Turtlebot3-and-SLAM/blob/main/05-installing%20the%20simulation%20packag.PNG)
  
  
  To launch the simulation world in empty environment use these commands
  
  
  ```bash
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
  ```
  
  ![06-empty environment](https://github.com/AlolyanRoaa/Creating-a-map-using-Turtlebot3-and-SLAM/blob/main/06-empty%20environment.PNG)
  
  
  in TurtleBot3 world environment
  
  ```bash
  $ export TURTLEBOT3_MODEL=burger
  $ roslaunch turtlebot3_gazebo turtlebot3_world.launch
   ```
   
   ![07-TurtleBot3 world environment](https://github.com/AlolyanRoaa/Creating-a-map-using-Turtlebot3-and-SLAM/blob/main/07-TurtleBot3%20world%20environment.PNG)
   
   
   ### Operate TurtleBot3 Burger in Gazebo
   
   
   in a new terminal after running gazebo in the previous part, run the following command to teleoperate the turtleBot3.
   
   ```bash
   $ export TURTLEBOT3_MODEL=burger
   $ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
  ```
  
  ![08-teleoperate the turtleBot3](https://github.com/AlolyanRoaa/Creating-a-map-using-Turtlebot3-and-SLAM/blob/main/08-teleoperate%20the%20turtleBot3.jpg)
  
  *Note: now you can control turtleBot3 burger using keyboard keys As shown above.*
  
  
  
  ## 3-creating the Map with SLAM
  
  first gazebo must be running using the previous command. and in new terminal run turtleBot3 SLAM.
  
  
 ```bash
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
  ```
  
  ![09-run turtleBot3 SLAM](https://github.com/AlolyanRoaa/Creating-a-map-using-Turtlebot3-and-SLAM/blob/main/09-run%20turtleBot3%20SLAM.PNG)
  
  
  create new terminal and run the teleoperation command in part 2.
  
  ![10-teleoperation command to draw the map](https://github.com/AlolyanRoaa/Creating-a-map-using-Turtlebot3-and-SLAM/blob/main/10-teleoperation%20command%20to%20draw%20the%20map.PNG)
  
  
  now you can move turtleBot3 burger with keyboard to create the map 
   
  finally create new terminal ans save the map.
  
  ```bash 
  $ rosrun map_server map_saver -f ~/map
  ````
  
  ![11-done of creating the map and saving command](https://github.com/AlolyanRoaa/Creating-a-map-using-Turtlebot3-and-SLAM/blob/main/11-done%20of%20creating%20the%20map%20and%20saving%20command.PNG)
  
  
  
  ## Purpose of the map
  This map will help us navigate the robot and move from a location to destination in a given environment.
  
  ![12-tha mapping that we made](https://github.com/AlolyanRoaa/Creating-a-map-using-Turtlebot3-and-SLAM/blob/main/12-tha%20mapping%20that%20we%20made.PNG)
  
  





  
