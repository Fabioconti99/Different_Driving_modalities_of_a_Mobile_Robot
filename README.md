#  [Research_Track_1](https://unige.it/en/off.f/2021/ins/51201.html?codcla=10635) , [Robotics Engineering](https://courses.unige.it/10635) ([UNIGE](https://unige.it/it/)) : Final assignement
## Robot Operating System <img height="30" src="https://github.com/Fabioconti99/RT1_Assignment_2/blob/main/images/ros.png"> & Python <img height="30" src="https://github.com/Fabioconti99/RT1_Assignment_1/blob/main/images/python.png">
### Professor. [Carmine Recchiuto](https://github.com/CarmineD8)


--------------------
Project objectives
--------------------
This project is about the development of a software architecture for the control of a mobile robot.
The architecture should be able to get the user request, and let the robot execute one of the following behaviors (depending on the user's input):

* **Autonomously reach an x,y coordinate** inserted by the user,
* Let the user **drive** the robot **with the keyboard**,
* Let the user drive the robot while assisting them to **avoid collisions**.

The user will be able to change the driving modality through a **UI interface**:
* Pressing **[1]**, the interface will ask the user to insert the *x* and *y* coordinates. The robot will eventually try to drive autonomously to those coordinates.
* Pressing **[2]**, the UI will activate a simple *teleop_key* interface that will let the user drive the robot with keyboard inputs.
* Pressing **[3]**, the UI will add an extra a driving assistency feature to the previous modality. The program will assist the user through a **collision avoidence** layer. This feature will prevent the robot from hitting the walls when getting too close to them. 

The software will rely on the `move_base` and `gmapping` packages for localizing the robot and planning the motion.

* The `move_base` package will provide an implementation of an *action* that, given a goal in the world, the robot will attempt to reach it with a mobile base. (Actions are services which are notexecuted automatically, and thus may also offer some additional tools such as the possibility of cancelling the request. 
* The `gmapping` pakage contains the algorithm based on a *particle filter* (approach to estimate a probability density) needed for implementing Simultaneous Localization and Mapping (SLAM). Needed by the `gmapping` package. 

The package will be tested on a simulation of a mobile robot driving inside of a given environment. The simulation and visualization are run by the two following programs: 

* **Rviz**: which is a tool for ROS Visualization. It's a 3-dimensional visualization tool for ROS. It allows the user to view the simulated robot model, log sensor information from the robot's sensors, and replay the logged sensor information. By visualizing what the robot is seeing, thinking, and doing, the user can debug a robot application from sensor inputs to planned (or unplanned) actions.

* **Gazebo**: which is the 3D simulator for ROS. 

Picture of the **Gazebo Enviroment**:


![Schermata 2022-02-12 alle 00 30 58](https://user-images.githubusercontent.com/91262561/153684461-be2e2074-d17e-4f01-acf0-dd8481ec07d5.png)



Picture of the **Robot inside the enviroment**:


![IMG_0034](https://user-images.githubusercontent.com/91262561/153683955-682e4ca8-9282-4c45-98f9-52ef0e3a186b.GIF)


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

The *Python* scripts I developed define a **user interface** that will let the user switch between driving modalities.
The **four scripts** provided are the following: 

* UI.py: Which represents a kind of menu where the user can switch between driving modalities.

* go_to_desired_pos: This script implements an *Action* client-service communication that will manage to drive the robot to a chosen position in the environment.

* my_teleop_twist_keyboard: Which will let the user directly drive the robot using keyboard inputs.

* teleop_avoid.py: Which will let the user directly drive the robot with keyboard inputs.

* avoidence.py: The last modality adds an obstacle avoidance capability to the second modality thanks to the custom message `Avoid.msg`. This added feature will prevent the user to drive the robot into a wall.

## Running the simulation

To Run the simulation easily I added two *launch files* to the package. 

* launch_nodes.launch: That will launch the previously mentioned nodes through the use of the *Xterm* terminal. It will also initialize some **parameters** that will be used by the nodes during execution.

* launch_All.launch: that will *include* all the launch files needed for running the whole simulation all at once.

Run the following command from the shell to activate all the nodes:
```bash
roslaunch final_assignment launchAll.launch
```

This kind of execution needs the Xterm terminal to be installed. If it's not already installed you can download it with the following shell command:

```bash

sudo apt-get install -y xterm
```
------
NODES Description
-------------------

## UI node: UI.py

This node controls the robot's driving capabilities inside the environment. The **UI function** will command the robot to drive with a certain **Modality** inside the Gazebo map. Thanks to this node, the User will interact with the simulation choosing the driving mode through certain keyboard inputs.

Driving modalities related to their keyboard inputs:

* The keyboard input **[0]** resets the current driving modality.
* The keyboard input **[1]** will start the autonomous drive towards a certain location in the map chosen by the user.
* The keyboard input **[2]** will start a simple teleop-key interface.
* The keyboard input **[3]** will add to the previous interface an avoidance layer.

Thanks to the `launch_nodes.launch` launch file, I added **three parameters** to the project for managing the *different activation state* of all the nodes involved in the project.
The three parameters are:

* **Active**: This parameter manages the current state of the project's ROS node chain. Once the program is launched, the parameter is set to be in *idle state* (0 states). In the beginning, one of the nodes will be in its active state. The UI node is capable of managing the change of the value of this parameter thanks to the retrieved user input. A simple legend will tell the user what button to press for running a certain driving modality. The user input will change the value of the parameter and all the nodes will either keep their current idle state or switch to a running state. An If-Statement inside every node manages this modality switch.

* **Posion X and Position Y**: Also, these two parameters are retrieved by an input user managed in the UI node. Once the user selects the **first modality [1]** the UI interface will also ask for an X and Y coordinate. This data represents the position we want the robot to go. If the user wants to stop the robot's motion, it is sufficient to either input another driving modality or set the project idle state.
The UI node will also keep the user updated on the current modality thanks to the on-screen messages sent at every state switch. Some flags will keep track of the current modality based on the UI inputs.

The follwing graph rappresents the whole UI structure:

![ Diagramma_vuoto](https://user-images.githubusercontent.com/91262561/153684103-0222448b-aeac-4925-bcea-8fbc7cdf96e8.png)



------
## Autonomous drive mode: go_to_desired_pos.py

This node implements the autonomous driving capability. The script exploits an **action client** (*actionlib* library) instance to establish direct communication with the mobile robot and set and cancel location goals.

The Action Client-Service communicate via a "ROS Action Protocol", which is built on top of ROS messages. The client and server then provide a simple API for users to request goals (on the client side) or to execute goals (on the server side) via function calls and callbacks. 
Through out the coding of this node I implemented only the *Actionclient* side of the whole structure using the already existing server of the following action messages:

* `MoveBaseAction`
* ` MoveBaseGoal`

The following picture shows a graphical rappresentation of the ROS-Action protocol: 


<img width="674" alt="Schermata 2022-02-11 alle 11 33 10" src="https://user-images.githubusercontent.com/91262561/153684323-a75025f3-28f4-4ad2-82e6-bd0a057d9c3a.png">



* *goal*: used to send new goals to server
* *cancel*: used to send cancel requests to server
* *status*: used to notify clients on the current state of every goal in the system
* *feedback*: used to send clients periodic auxiliary information for a goal
* *result*:  used to send clients one-time auxiliary information about the completion of a goal

For the client and server to communicate, I should define a few messages on which they communicate. This defines the Goal, Feedback, and Result messages with which clients and servers communicate. throughout the coding, I only used the Goal message because that was the one message needed for fulfilling the project aim. 

Thanks to the Actionlib feature, an ActionServer receives the goal message from an ActionClient. In the case of my project, the goal is to move the robot's base position. The goal would be a MoveBaseGoal message that contains information about where the robot should move to in the world. For controlling all the robot positions in space, the goal would contain the *target_pose* parameters (stamp, orientation, target position, etc).

 ### Main functions used
 
The following function sets the standard instances of the position I want to achieve and it also initializes the client-side of the action:

```python
def action_client_init():

    global client 
    global goal 
    
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction) # Initialization of the action client.
    client.wait_for_server()                            # Waiting for the server to get ready.
    
    goal = MoveBaseGoal()                         # Initialization of the goal message.
    goal.target_pose.header.frame_id = "map"            # Setting up some parameters of the goal message.
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.orientation.w = 1.0
    
# Call Back used for setting up a timeout to the robot's current task.
def my_callback_timeout(event):
    if active_==1:
        print ("\033[1;31;40m Goal time expired\033[0;37;40m :" + str(event.current_real)+st)
        print("The robot didn't reach the desired position target within a 1min time span\n")
        rospy.set_param('active', 0)
```

Once the node gets to its active state, the retrieved info on the goal position will be retrieved from the newly set parameters and inserted inside the goal message structure. This operation is taken care of by the following "set-goal" function: 

```python
def action_client_set_goal():

    goal.target_pose.pose.position.x = desired_position_x
    goal.target_pose.pose.position.y = desired_position_y
    print("\033[1;33;40m START AUTONOMOUS DRIVE"+st+"\033[0;37;40m \n")
    client.send_goal(goal,done_cb)
```
The following image shows the Rviz graphical interface once the goal is set:



![Schermata 2022-02-12 alle 00 31 15](https://user-images.githubusercontent.com/91262561/153684513-037947ca-4470-48de-8319-53d2aab4cd07.png)


The argument `done_cb` of the `send_goal` function is a special call-back function needed for retrieving info on the goal-reaching *status*. This function retrieves info directly from the server-side. There are many different values associated with the status parameter during the final portion of the execution. The only one used in the code is the *status 3* related to the goal achievement:

```python
def done_cb(status,result):
    
    global flag_goal
    
    if status==3:
        print("\033[1;34;40m goal achived!"+st+"\033[0;37;40m \n")
        flag_goal = 1
```

Since the server provides the output of this function, I choose to ignore any other given status because I wouldn't have any direct control over it. An example of this is "the ending on a *timeout*" feature. There exists a status for retrieving a *timeout* ending for the robot. I ignored it because the actual time is set directly by the server.
The following function sets a timer expiration goal that is locally set to 1 minute. Once it expires it will automatically cancel the goal.


```python
def my_callback_timeout(event):
    if active_==1:
        print ("\033[1;31;40m Goal time expired\033[0;37;40m :" + str(event.current_real)+st)
        print("The robot didn't reach the desired position target within a 1min time span\n")
        rospy.set_param('active', 0)
        
```

Thorough out the whole execution, thanks to the following callback, the program will print the actual position on screen with a 10hz rate. The actual position is not retrived by the *action feedback* but from a subscription to the odometry topic `/odom`. 

```python
def clbk_odom(msg): 
    global position_
    position_ = msg.pose.pose.position
    
```

Picture of the standard GUI window of the first modality:


<img width="312" alt="Schermata 2022-02-12 alle 01 01 07" src="https://user-images.githubusercontent.com/91262561/153687536-b5add6e5-5d3f-4e2a-8362-17928206ef0e.png">


The normal `cancel_goal` is activated once the robot gets back into its idle state. The cancel call is managed by all the flags that determine the current state of the process.

```python
# The active value is not set to 1
else:
    # Initial idle state 
    if flag == 0 and flag_2==0:
        
        print("\033[1;31;40m STOP MODALITY 1 \033[0;37;40m \n")
        flag = 1
    
    # Idle state the node will get to once the robot gets stopped by the user.
    if flag == 0 and flag_2==1:
        
        # Flag needed to know if the goal is reached or not
        if flag_goal==1:
            # If the goal is reached I will not cancel the goal because. 
            print("\033[1;31;40m STOP MODALITY 1 "+st+"\033[0;37;40m")
            flag = 1
            flag_2 = 0
            flag_goal = 0
    
        else:
            # If the goal is not reached once the user switches modality or the time expires with the time-out.
            print("\033[1;31;40m GOAL CANCELED, STOP MODALITY 1 "+st+"\033[0;37;40m")
            client.cancel_goal()
            flag = 1
            flag_2 = 0
    
```

If the `active` param is set to a value different than 1, the program will at first execute one of the if-sections and it will later just idle waiting for `the active` param to get to 1. 


------
## KeyBoard input drive mode: teleop_avoid.py

The script is based on the standard ROS teleop_twist_keyboard.py.
This node is constantly checking which keys are pressed on a PC keyboard and based on the pressed keys, publishes twist messages on the `/cmd_vel` topic. Twist message defines what should be the linear and rotational speeds of a mobile robot.

 ### Main functions used
 
I added some functions and changes to the code to merge it with the `avoidence.py` publisher and to manage the alternity of the activation state.
Just like all the other programs, I had to manage the alternity of the *idle* to the *active* state through the use of an If-statement. Since the node also includes the functionalities of the third modality, the statement will set the node to an active modality if the `active` param is either set to 3 or 2.
I added a new function named `stop_motion` to the `PublishThread` class. This function will make the robot stop once the driving modality gets switched. The function will set the linear and angular velocity to 0 with the `twist` message through the `/cmd_vel` topic. 

```python
def stop_motion(self):
    twist = Twist()
    # Publish stop message when thread exits.
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
            
    self.publisher.publish(twist)
```

The node **subscribes** to the custom topic `custom_controller` implemented for publishing the `Avoid. msg` message containing info about the walls surrounding the robot. The callback subscribed sets some local variables equal to the published fields of the custom message. The following uses these variables function to change some keyboards inputs to prevent the user to drive the robot into walls. 

```python
def new_dict(dictionary):

    global ok_left
    global ok_right
    global ok_straight
    
    # If any of the flags for checking if the wall are turned on in any combinations, the function will disable the corrisponding directions command.
    if not ok_straight == 1 and not ok_right == 1 and not ok_left == 1:
        dictionary.pop('i')
        dictionary.pop('j')
        dictionary.pop('l')
        
    elif not ok_left == 1 and not ok_straight == 1 and ok_right == 1:
        dictionary.pop('i')
        dictionary.pop('j')
        
    elif ok_left == 1 and not ok_straight == 1 and not ok_right == 1:
        dictionary.pop('i')
        dictionary.pop('l')
        
    elif not ok_left == 1 and ok_straight == 1 and not ok_right == 1:
        dictionary.pop('l')
        dictionary.pop('j')
        
    elif ok_left == 1 and not ok_straight == 1 and ok_right == 1:
        dictionary.pop('i')
        
    elif not ok_left == 1 and ok_straight == 1 and ok_right == 1:
        dictionary.pop('j')
        
    elif ok_left == 1 and ok_straight == 1 and not ok_right == 1:
        pdictionary.pop('l')
```

A **dictionary** m manages the keyboard input set by:
A dictionary is *python* unordered and changeable collection of data values that holds key-value pairs. Each key-value pair in the dictionary maps the key to its associated value making it more optimized.
In the standard `teleop_twist_keyboard`, a dictionary is used to collect the buttons for all the possible robot's movements. In my version of the node, some of these keys are omitted to code an easier implementation of the avoidance feature. The following instance is the dictionary used in the node:

```python
# Dictionary for moving commands
moveBindings = {
        'i':(1,0,0,0),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'k':(-1,0,0,0),
    }
```
The associated values optimize the UI making it easier for the user to interact with the simulation. Thanks to these values, the node will public the right values to the `/cmd_vel` topic for making the robot move accordingly with the input.

The following table shows the commands related to each keyboard input:


<img width="521" alt="Schermata 2022-02-11 alle 21 25 55" src="https://user-images.githubusercontent.com/91262561/153684181-01d41767-99be-40df-9fa0-67760e7d7c83.png">


The `new_dict()` function uses the `pop` command to directly remove some keys from the dictionary. The removal will happen accordingly to the values retrieved by the previously mentioned callback to the `custom_controller` topic. The values retrieved by the teleop node are relocated in the following local variables:

* `ok_right`:
    * 1 = the wall is not close to the right of the robot. The user will be able to turn right. 
    * 0 = the wall is close to the right of the robot. The user will not be able to turn right. 

* `ok_left`:
    * 1 = the wall is not close to the left of the robot. The user will be able to turn left. 
    * 0 = the wall is close to the left of the robot. The user will not be able to turn right.
    
* `ok_straight`:
    * 1 = the wall is not close to the front of the robot. The user will be able to drive straight. 
    * 0 = the wall is close to the front of the robot. The user will not be able to drive straight.
    
The following scheme shows all the combinations that the program considers for the wall avoidence:


![Schermata 2022-02-11 alle 21 07 11](https://user-images.githubusercontent.com/91262561/153684224-6a05a9d8-8478-4a84-b2cf-31c374bfd92b.png)



At every cycle, the dictionary will switch to a temporary one that will consider the just "popped" commands.

------
## Avoidence feature: avoidence.py


This node aims to activate a security feature for driving with the teleop_key modality. Thanks to the **subscription** to the `/laser_scan` topic, the node will be able to get info about the robot's surroundings. The subscription to the topic will give back the `ranges[0,720]` array to the subscribed callback. This data structure contains the distance values between the robot and the surrounding walls for a span of 180º degrees in front of the robot. The array simulates the info that a set of lasers would retrieve in an actual environment.
The node will later elaborate the data acquired to publish it on the `custom_controller` custom topic through the `Avoid.msg` custom message.

### Main functions used

`cb_avoid(msg)` is the callback function used to acquire and manage the data from the `/lase_scan` subscription. Once the callback retrieves the `ranges[]` array, the following 3 sub-ranges divide the data structure  as follows:
* From 0 to 143: which represents the right side of the scanned area.
* From 288 to 431: which represents the front side of the scanned area.
* From 576 to 719: which represents the left side of the scanned area.

the following picture gives a graphical rappresentation of the 3 sub arrays:


<img width="851" alt="Schermata 2022-02-12 alle 00 08 48" src="https://user-images.githubusercontent.com/91262561/153684283-7ee3d999-50b8-4779-b83c-d590a64a4b2b.png">



To the local variables `right`, `front`, and `left` are assigned the smaller value retrieved by their correspondent array. If one of these values is smaller than 1 an if-statement will set the correspondent `ok_(right, front, left)` global variable to 0. the `custom_controller` custom topic will later receive these variables through the `Avoid. msg` custom message by the `main` function.
If the `active` parameter is not set to 3, the node will just publish the values 1 to simulate the idle state of the modality.

```python
def cb_avoid(msg):

    global ok_left
    global ok_right
    global ok_straight
    
    active_=rospy.get_param("/active")        # Assignment of the active param value to a local variable.
    if active_ == 3:
        
        right = min(msg.ranges[0:143])      # right checking laser span.
        front = min(msg.ranges[288:431])    # front checking laser span.
        left = min(msg.ranges[576:719])     # left checking laser span.
        
        if right < 1.0:         # If the robot is close to the right of the robot.
            ok_right = 0
        else:
            ok_right = 1
        if front < 1.0:         # If the robot is close to the front of the robot.
            ok_straight = 0
        else:
            ok_straight = 1
        if left < 1.0:          # If the robot is close to the left of the robot.
            ok_left = 0
        else:
            ok_left = 1
    else:                       # Let all the direction good to go if the modality 3 is turned off.
        ok_right = 1
        ok_straight = 1
        ok_left = 1
```

The `main` function is used only to initialize the publisher and the subscriber's instances and publish the avoidance message on the `custom_contrller` topic. The while loop will spin at a 10hz rate thanks to the `sleep` function. 

```python
def main():

    global ok_left
    global ok_right
    global ok_straight
    
    pub = rospy.Publisher('custom_controller', Avoid, queue_size=10)    # Publisher.
    rospy.init_node('avoidence')                                        # Initialization of the node.
    sub = rospy.Subscriber('/scan', LaserScan, cb_avoid)                # Sub to the '/scan' topic.
    rate = rospy.Rate(5)                                                #10hz
    
    pub_msg = Avoid()
    while not rospy.is_shutdown():
        pub_msg.left = ok_left          # Assigning the messages fields
        pub_msg.right = ok_right        # Assigning the messages fields
        pub_msg.front = ok_straight     # Assigning the messages fields
        
        pub.publish(pub_msg)        # publishing the messages fields
        rate.sleep()                # 10hz delay.

```

The whole ROS nodes net is independent of this node. In case the node wouldn't start, the project would still execute fine.



RQT-Graph
--------------------

<img width="983" alt="Schermata 2022-02-12 alle 00 47 12" src="https://user-images.githubusercontent.com/91262561/153687510-a76bef7d-8bf8-467a-bc9b-d5e7da678065.png">



Possible improvements
-----------------

* As a future improvement, I'd like to implement a different interface between the *Action-client* and *Action-service* communication. The *Action-service* could receive the position in the environment through the use of **feedback** Callback. The **status** Callback could have distinguished more status values and it could have handled the different situations that could refine the autonomous driving capability.
* It is also possible to implement a custom Action. I could use a different service to implement a finer control of the robot through clients' requests.
* Avoidence should be written as "avoidance".


Conclusions
-----------------
To take care of all the project's requests, I choose to manage the code's structure with modular logic. Thanks to this approach, I was able to reach the end of the assignment with a schematic structure of the project concerning only 4 extra Ros nodes. Thanks to the parameters introduced by the launch file, the robot can easily change driving modalities and quickly switch between them. The whole implementation makes the robot capable of driving around the environment following the requested rules.

This project was my second approach to the ROS1 workflow. Working on this assignment, I gained knowledge about the concepts of ROS launch files, ROS actions, managing ROS parameters and implementing *messages* and building a modular and organized *package* structure. 

(**credit to:** Luca Predieri, Francesco Pagano, Alessandro Perri, Matteo Carlone for the implementetion of the dctionary pop function of the avoidance feature)
