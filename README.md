## About

<!-- GETTING STARTED -->
## Getting Started

All packages in this repository have been developed, executed and tested in an Ubuntu 20.04 machine with ROS2.0 Foxy. Please find below all the required steps to set-up a ROS2.0 Foxy environment in Ubuntu and install the Robot Simulation packages.



<p align="right">(<a href="#top">back to top</a>)</p>


<!-- ROS2.0 Packages: Explanation -->
## ROS2 Packages 

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



<p align="right">(<a href="#top">back to top</a>)</p>


<p align="right">(<a href="#top">back to top</a>)</p>
