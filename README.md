#  [Research_Track_1](https://unige.it/en/off.f/2021/ins/51201.html?codcla=10635) , [Robotics Engineering](https://courses.unige.it/10635) ([UNIGE](https://unige.it/it/)) : Final assignement
## Robot Operating System <img height="30" src="https://github.com/Fabioconti99/RT1_Assignment_2/blob/main/images/ros.png"> & Python <img height="30" src="https://github.com/Fabioconti99/RT1_Assignment_1/blob/main/images/python.png">
### Professor. [Carmine Recchiuto](https://github.com/CarmineD8)


--------------------
Project objectives
--------------------
This project is about the development of a software architecture of the control of a mobile robot.
The architecture should be able to get the user request, and let the robot execute one of the following behaviors (depending on the user's input):

* **Autonomously reach an x,y coordinate** inserted by the user,
* Let the user **drive** the robot **with the keyboard**,
* Let the user drive the robot assisting them to **avoid collisions**.

The user will be able to change the driving modality through a **UI interface**:
* pressing **[1]** the interface will ask the user to insert the *x* and *y* coordinates. The robot will eventually try to drive autonomously to those coordinates.
* pressing **[2]** the UI will activate a simple *teleop_key* interface that will let the user drive the robot with keyboard inputs.
* pressing **[3]** the UI will activate a similar modality to the second one but the user will be assisted by a **collision avoidence** layer that will prevent the robot to hit the walls if it gets too close to them. 

The software will rely on the `move_base` and `gmapping` pakages for localizing the robot and plan the motion.

* The `move_base` package will provide an implementation of an *action* that, given a goal in the world, the robot will attempt to reach it with a mobile base. (Actions are services which are notexecuted automatically, and thus may also offer some additional tools such as the possibility of cancelling the request. 
* The `gmapping` pakage contains the algorithm based on a *particle filter* (approach to estimate a probability density) needed for implementing Simultaneous Localization and Mapping (SLAM). Needed by the `gmapping` package. 

The package will be tested on a simulation of a mobile robot driving inside of a given enviroment. The simulation and visualization are run by two following programs: 

* **Rviz**: which is a tool for ROS Visualization. It's a 3-dimensional visualization tool for ROS. It allows the user to view the simulated robot model, log sensor information from the robot's sensors, and replay the logged sensor information. By visualizing what the robot is seeing, thinking, and doing, the user can debug a robot application from sensor inputs to planned (or unplanned) actions.

* **Gazebo**: which is the 3D simulator for ROS. 

Picture of the **Gazebo Enviroment**:

![alt text](https://github.com/Fabioconti99/RT1_Assignment_2/blob/main/images/map.png) 

Picture of the **Robot inside the enviroment**:

![alt text](https://github.com/Fabioconti99/RT1_Assignment_2/blob/main/images/map.png) 

--------

Installing and running
----------------------
The simulation requires the following steps before running:

* A [ROS Noetic](http://wiki.ros.org/noetic/Installation) installation,

* The download of the `slam_gmapping` package form the *Noetic* branch of the [teacher's repository](https://github.com/CarmineD8/slam_gmapping.git )

Run the following command from the shell:
```bash
git clone https://github.com/CarmineD8/slam_gmapping.git
```

* The download of the **ROS navigation stack** (run the following command from the shell)

Run the following command from the shell:
```bash
sudo Apt-get install ros-<your_ros_distro>-navigation
```

* And the and the clone of the [Current repository](https://github.com/Fabioconti99/RT1_Assignment_3 ). After downloading the repository, you should take the `final_assignment` directory included in the repo and place it inside the local workspace directory.

Run the following command from the shell:
```bash
git clone https://github.com/Fabioconti99/RT1_Assignment_3
```

The *Python* scripts I developed define a **user inteface** that will let the user switch between driving modalities.
The **four scripts** provided are the following: 

* UI.py: Which rappresents a kind of menu where the user can switch between driveing modalites.

* go_to_desired_pos: This script implements an *Action* client-service comunication that will manage to drive the robot to a choosen position in the enviroment.

* my_teleop_twist_keyboard: Which will let the user directly drive the robot with keyboard inputs.

* teleop_avoid.py: The last modality implements a similar interface to the previous one but it also adds an obstacle avoidence capability. this added feature will prevent the user to drive the robot into a wall.

## Running the simulation

To Run the simulation easily I added two *launch files* to the package. 

* launch_nodes.launch: That will launch the previously mentioned nodes through the use of the *Xterm* terminal. It will also initialize some **parameters** that will be used by the nodes during execution.

* launch_All.launch: that will *include* all the launch files needed for running the whole simulation all at once.

Run the following command from the shell to activate all the nodes:
```bash
roslaunch final_assignment launchAll.launch
```

This kind of execution needs the Xterm terminal to be installed. If the it's not already installed you can download it with the following shell command:

```bash

sudo apt-get install -y xterm
```

