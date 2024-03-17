## About



```bash
apt-get update
apt-get install -y nvidia-docker2
systemctl restart docker


xhost +local:docker
docker run -it --privileged --net="host" -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --gpus all   --env="QT_X11_NO_MITSHM=1" ros:foxy-ros-base bash -c  "sleep 1;source /root/.bashrc ; ros2 launch panda_ros2_moveit2 panda_interface.launch.py; exec bash"

```
<!-- GETTING STARTED -->
## Getting Started

All packages in this repository have been developed, executed and tested in an Ubuntu 20.04 machine with ROS2.0 Foxy, Instillation is not generic and ROS-DISTRO specific therefore wrapped in a docker image.












<h1 align="center">Nodogoro Simulation Task</h1>

----
## 📝Table of Contents
- [📝Table of Contents](#table-of-contents)
- [Code Docs](#code-docs)
- [Solution](#solution)
- [Instillation](#instillation)
  - [Ros2 Humble](#ros2-humble)
  - [Docker](#docker)
- [🎈 Usage ](#-usage-)
  - [Ros2 Humble](#ros2-humble-1)
  - [Docker](#docker-1)
- [✍️ Authors ](#️-authors-)
- [🏁 Getting Started ](#-getting-started-)
  - [Problem](#problem)
  - [Shortest Path (A\*)](#shortest-path-a)
  - [Controller (navigation\_control)](#controller-navigation_control)
  - [Map (map\_publisher)](#map-map_publisher)
  - [Odometry](#odometry)
    - [Calculated with Velocity Vectors](#calculated-with-velocity-vectors)
    - [Simulation](#simulation)
  - [Simulation](#simulation-1)
- [Directory Tree](#directory-tree)

## Code Docs
https://omarwalid96.gitlab.io/zal_robotics_engineer_task/

> [!NOTE]
> This solution is running on a docker image, a GPU is needed to avoid simulation jittering.

## Solution
![Video](https://i.imgur.com/UYCaZkp.gif)
<img src="https://i.imgur.com/EWTWwQT.png" alt="Image" style="width:75%; height:75%;">
## Instillation


### Docker
1. install noVNC `docker pull theasp/novnc:latest`
2. run `docker run -d --rm --net=ros  --env="DISPLAY_WIDTH=3000" --env="DISPLAY_HEIGHT=1800" --env="RUN_XTERM=no" --name=novnc -p=8080:8080  theasp/novnc:latest`
3. opn viewer on any browser `http://localhost:8080/vnc.html`
4. install docker image `docker pull omarwalid96/ros2-zal-task`



## 🎈 Usage <a name="usage"></a>

- Gui opens and you can instantly send commands from **2d goal pose** <img src="https://i.imgur.com/gsQyRdB.png" alt="Image" style="width:75%; height:75%;">


### Docker
1. run `docker run -d --rm --net=ros  --env="DISPLAY_WIDTH=3000" --env="DISPLAY_HEIGHT=1800" --env="RUN_XTERM=no" --name=novnc -p=8080:8080  theasp/novnc:latest`
2. run `docker run  -it --net="ros"  --privileged omarwalid96/ros2-zal-task /bin/bash -c "sleep 2;cd /home/zal_robotics_engineer_task ; source /opt/ros/humble/setup.bash; source ./install/setup.bash; ros2 launch omni_bot zal_task.launch.py"`
3. connect to viewer on `http://localhost:8080/vnc.html`
<img src="https://i.imgur.com/ohAMiEX.png" alt="Image" style="width:75%; height:75%;">


## ✍️ Authors <a name = "authors"></a>

- [@omarwalid96](https://gitlab.com/omarwalid96) 


## 🏁 Getting Started <a name = "getting_started"></a>
### Problem 
Objective:
Design and implement a simulation in ROS/Gazebo that demonstrates a robotic arm manipulating a specific ingredient (e.g., an apple). The simulation must include picking up the apple from a specified location, moving it through a predetermined path, and then placing it down at a target location without causing damage to the ingredient.
- Environment Setup
- Robotic Arm and Apple Model
- Simulation Tasks

### Environment Setup
#### Kitchen:
Used Blender to design a kitchen and export a `.dae` file that can be used in gazebo.
- Reduced mesh size in Blender 
- Specified physical paramters such as mass and inertia of model
- Linked model to the world of the robot arm
#### Apple:
- Got a ready `.stl` file of an apple from the internet 
- Specified physical paramters such as mass and inertia of model
- Set model collision as false so the arm can grab the apple
#### Robot Arm:
- Used Panda Robot arm
- Added a padestel so the robot can be in a good location relative to the tabletop
- Specified physical paramters of the padestale relative to robot size and maximum torque the robot could achive on it and added a safety margin to make the robot stable

### Simulation Tasks
With help of multiple packages the tasks were met (packages will be disscussed below)
1. Approached the apple location relative to the ennvironment
2. Opened Gripper 
3. Approached Apple with lower speed
4. Attached the apple to gripper link 
5. Closed the gripper relative to apple size and with certain effort
6. Moved apple across the table
7. Approached final goal with lower speed
8. Detached the apple and Opened Gripper.
9. Retracted with slow speed.
10. Home Position


<!-- ROS2.0 Packages: Explanation -->
## ROS2 Packages 

### Moveit2 
https://moveit.picknik.ai/main/index.html
### ros2_data
This repository contains all the custom data structures that are used for the ROS2 Robot Actions/Triggers in ros2_RobotSimulation. Every single ROS2 Action that is used to trigger a robot movement refers to a .action data structure, which contains all the data needed to run that specidic action. 

### ros2_actions
The ros2_actions package gathers all the ROS2 Action Servers that execute ROS2 Robot Actions/Triggers. Every specific robot movement is contained in a ROS2 Action Server (.cpp script), which can be externally "called" (triggered) and returns some feedback after the execution.
### ros2_execution
It is a cool feature to be able to execute different ROS2 actions and trigger different movements in a Robot in Gazebo, but Robotics Applications are made of sequences that execute different commands one after the other. In a nutshell, that is what this ros2_execution package does. The ros2_execution.py script contains all the Action Clients that connect to the Action Servers generated in ros2_actions, and executes robot movements one after the other according to a pre-defined sequence, which is inputted as a .txt file.

One of the main advantages of using this ros2_execution package, combined with ros2_data/ros2_actions and the Robot Simulation packages contained in this repository, is that programs/sequences can be executed in the exact same way for different Robots, which is a completely novel feature and has been made possible thanks to the MoveIt!2 framework.



### ros2_grasping
Unfortunately, Gazebo and ROS2 do not provide an effective method to properly pick and place (manipulate) objects in simulation (if it exists, it has not been found). This feature is essential in order to test and simulate different applications, and that is the main reason why this ros2_grasping package has been created.

The attacher_action.py script contains a ROS2 Action Server that performs the task of attaching/detaching a specific object to a specific end-effector in Gazebo. Apart from the attaching functionality, the spawn_object.py script enables the user to spawn custom objects (contained in .urdf of .xacro files) to a Gazebo simulation environment. 



<p align="right">(<a href="#top">back to top</a>)</p>


