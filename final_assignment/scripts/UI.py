#!/usr/bin/python3


from std_srvs.srv import *
import math
import rospy




"""
.. module:: UI
 :platform: Unix
 :synopsys: Python module for user interface

.. moduleauthor:: Fabio Conti <s4693053@studenti.unige.it>

This is the ROS node which controls the robot's driving capabilities inside the environment. The *UI function* will command the robot to drive with a certain *Modality* inside the Gazebo map. Thanks to this node, the User will interact with the simulation choosing the driving mode through certain keyboard inputs.

H2 -- Driving modalities
========================

Driving modalities related to their keyboard inputs:

* The keyboard input *[0]* resets the current driving modality (idle state).
* The keyboard input *[1]* will start the autonomous drive towards a certain location in the map chosen by the user (:mod:`go_to_desired_pos`).
* The keyboard input *[2]* will start a simple teleop-key interface (:mod:`teleop_avoid`).
* The keyboard input *[3]* will add to the previous interface an avoidance layer (:mod:`avoidence`).

H2 -- Parameters
================

Thanks to the ``launch_nodes.launch`` launch file, I added *three parameters* to the project for managing the *different activation state* of all the nodes involved in the project.
The three parameters are:

* *Active*: This parameter manages the current state of the project's ROS node chain. Once the program is launched, the parameter is set to be in *idle state* (0 states). In the beginning, one of the nodes will be in its active state. The UI node is capable of managing the change of the value of this parameter thanks to the retrieved user input. A simple legend will tell the user what button to press for running a certain driving modality. The user input will change the value of the parameter and all the nodes will either keep their current idle state or switch to a running state. An If-Statement inside every node manages this modality switch.

* *Posion X and Position Y*: Also, these two parameters are retrieved by an input user managed in the UI node. Once the user selects the *first modality [1]* the UI interface will also ask for an X and Y coordinate. This data represents the position we want the robot to go. If the user wants to stop the robot's motion, it is sufficient to either input another driving modality or set the project idle state.
The UI node will also keep the user updated on the current modality thanks to the on-screen messages sent at every state switch. Some flags will keep track of the current modality based on the UI inputs.

"""

 

msg = """
\033[1;37;40m
Reading from the keyboard  and managing into the UI node!
---------------------------
Driving modalities:

-[0]: idle state.
-[1]: Autonomous drive towards a desired position.
-[2]: Teleop keyboard interface.
-[3]: Avoidence feature for modality 2.

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
\033[0;37;40m

"""

def main():
	"""
	The main function will constantly ask the user to choose the current driving modality for the movements of the robot. The following numbers rappresent the input the user has to insert in the UI to access a certain driving modality:

	* *0*: The robot will enter an IDLE STATE. In this modality no user input will be acceptet by the robot. The robot will not move until the modality will change.

	* *1*: This modality will activate an action that will drive automaticlly the robot toward a desired position. If the robot will not find the way to that certain coordinate position within 30 seconds from the beginning if the task, the driving will stop.

	* *2*: This driving option implements a simple teleop_key type of interface that will let the user drive the robot through the pressing of certain keyboard keys.  

	* *3*: The last modality adds an avoidence capability to the previous one. This added feature will prevent the user to drive the robot into a wall.

	"""
	
	flag = 0	# Flag used to print a cancel goal message whenever a button is pressed while the first modality is rolling.
	while not rospy.is_shutdown(): 
		command = int(input('\033[0;37;40m Choose modality: \n'))	# User input retrivial.
			
		# IDLE MODALITY
		if command == 0:
			
			# First modality calcel message.
			if flag == 1:
				print("Canceling goal")
				flag=0
				
			rospy.set_param('active', 0)			# Setting the parameter to 0.
			print("\033[1;35;40m Idle \033[1;31;40m")	# Printing the actual state.
				
		# AUTONOMOUS DRIVE
		elif command == 1:
		
			# First modality calcel message
			if flag == 1:
				print("Canceling goal")
				flag=0
				
			rospy.set_param('active', 0)		# Reset of the modality
			
			print("\033[1;32;40m Modality 1 is active, press '0' to cancel the target.")	# Print infos.
			print("\033[0;37;40m Where do you want the robot to go?")
			des_x_input = float(input("\033[1;37;40m Insert x coordinate: "))			# User Input for the x coordinate to reach.
			des_y_input = float(input("\033[1;37;40m Insert y coordinate: "))			# First modality calcel message.
			
			rospy.set_param('des_pos_x', des_x_input)			# Setting up the x-pos parameter with the retrived value.
			rospy.set_param('des_pos_y', des_y_input)			# Setting up the x-pos parameter with the retrived value.
			rospy.set_param('active', 1)					# Setting up the modality variable to the current driving modality.
			print("\033[1;32;40m Modality 1 is active.")
			flag = 1
			
		# KEYBOARD INTERFACE
		elif command == 2:
		
			# First modality calcel message
			if flag == 1:
				print("Canceling goal")
				flag=0
						
			rospy.set_param('active', 2)				# Set up of the second parameter.
			print("\033[1;33;40m Modality 2 is active.")
			
				
		# KEYBOARD INTERFACE + AVOIDENCE
		elif command == 3:
		
			# First modality calcel message
			if flag == 1:
				print("Canceling goal")
				flag=0
				
			rospy.set_param('active', 3) 				# Set up of the second parameter.
			print("\033[1;34;40m Modality 3 is active.")
				
				
		else:
			# Message that will be sent whenever the keyboard input is different form the previously mentioned ones.
			print("\033[1;31;40m Wrong key, try again.")


if __name__ == '__main__':
	print(msg)
	main()

