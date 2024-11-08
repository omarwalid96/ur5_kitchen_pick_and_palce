<h1 align="center">Nodogoro Simulation Task</h1>

----


## 📝Table of Contents

- [📝Table of Contents](#table-of-contents)
- [Code Docs](#code-docs)
- [Solution](#solution)
- [Instillation](#instillation)
  - [Docker](#docker)
- [🎈 Usage ](#-usage-)
  - [Docker](#docker-1)
- [✍️ Authors ](#️-authors-)
- [🏁 Getting Started ](#-getting-started-)
  - [Problem](#problem)
  - [Environment Setup ](#environment-setup-)
    - [Kitchen:](#kitchen)
    - [Apple: ](#apple-)
    - [Robot Arm: ](#robot-arm-)
  - [Simulation Tasks ](#simulation-tasks-)
- [ROS2 Packages ](#ros2-packages-)
  - [Moveit2](#moveit2)
  - [ros2\_data](#ros2_data)
  - [ros2\_actions](#ros2_actions)
  - [ros2\_execution](#ros2_execution)
  - [ros2\_grasping](#ros2_grasping)


## Code Docs
https://nodogro-test-omarwalid96-d971e11d58703eff12cd158faee6d4b30ed530.gitlab.io

> [!NOTE]
> This solution is running on a docker image, a GPU is needed to avoid simulation jittering.

## Solution 
Click on photo to see video to solution
[![Watch the video](ur5_apple.png)](https://i.imgur.com/mRarmfo.mp4)
## Instillation

### Docker
1. install docker image `docker pull omarwalid96/nodogoro-pick-and-place`
2. install nvidia docker link 
```bash 
apt-get update
apt-get install -y nvidia-docker2
systemctl restart docker
```
3. Give access to system Display from Xserver `xhost +local:docker`
    

## 🎈 Usage <a name="usage"></a>

- Gui opens and you can instantly works after **30 seconds** from boot.

### Docker
1. run `docker run -it --privileged --net="host" -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --gpus all   --env="QT_X11_NO_MITSHM=1" omarwalid96/nodogoro-pick-and-place`


## ✍️ Authors <a name = "authors"></a>

- [@omarwalid96](https://gitlab.com/omarwalid96) 
- [omarwalid@outlook.com](omarwalid@outlook.com)


## 🏁 Getting Started <a name = "getting_started"></a>

### Problem 
Objective:
Design and implement a simulation in ROS/Gazebo that demonstrates a robotic arm manipulating a specific ingredient (e.g., an apple). The simulation must include picking up the apple from a specified location, moving it through a predetermined path, and then placing it down at a target location without causing damage to the ingredient.
- Environment Setup
- Robotic Arm and Apple Model
- Simulation Tasks

### Environment Setup <a name = "Environment_Setup"></a>

#### Kitchen:
Used Blender to design a kitchen and export a `.dae` file that can be used in gazebo.
- Reduced mesh size in Blender 
- Specified physical paramters such as mass and inertia of model
- Linked model to the world of the robot arm

#### Apple: <a name = "Apple"></a>
- Got a ready `.stl` file of an apple from the internet 
- Specified physical paramters such as mass and inertia of model
- Set model collision as false so the arm can grab the apple

#### Robot Arm: <a name = "Robot_Arm"></a>
- Used Panda Robot arm
- Added a padestel so the robot can be in a good location relative to the tabletop
- Specified physical paramters of the padestale relative to robot size and maximum torque the robot could achive on it and added a safety margin to make the robot stable

### Simulation Tasks <a name = "Simulation_Tasks"></a>
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

## ROS2 Packages <a name = "ROS2_Packages"></a>

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


