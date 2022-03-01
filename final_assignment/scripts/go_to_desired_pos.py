#!/usr/bin/env python

"""
.. module:: go_to_desired_pos
 :platform: Unix
 :synopsys: Python node for robot's autonomous driving capapilities.

.. moduleauthor:: Fabio Conti <s4693053@studenti.unige.it>

Subscribes to:
 /nav_msgs/odometry topic where the simulator publishes the robot position.

Action:
 MoveBaseAction
 MoveBaseGoal
 

This node implements the autonomous driving capability. The script exploits an *action client* (*actionlib* library) instance to establish direct communication with the mobile robot and set and cancel location goals.

The Action Client-Service communicate via a "ROS Action Protocol", which is built on top of ROS messages. The client and server then provide a simple API for users to request goals (on the client side) or to execute goals (on the server side) via function calls and callbacks. 
Through out the coding of this node I implemented only the *Actionclient* side of the whole structure using the already existing server of the following action messages:

* ``MoveBaseAction``
* ``MoveBaseGoal``

For the client and server to communicate, I should define a few messages on which they communicate. This defines the Goal, Feedback, and Result messages with which clients and servers communicate. throughout the coding, I only used the Goal message because that was the one message needed for fulfilling the project aim. 

Thanks to the Actionlib feature, an ActionServer receives the goal message from an ActionClient. In the case of my project, the goal is to move the robot's base position. The goal would be a MoveBaseGoal message that contains information about where the robot should move to in the world. For controlling all the robot positions in space, the goal would contain the *target_pose* parameters (stamp, orientation, target position, etc).

"""

# libraries imports 
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import time
import math
import rospy


active_ = rospy.get_param('active')
"""
Parameter retrived for keeping track of the current driving modality.
"""
desired_position_x = rospy.get_param('des_pos_x')
"""
Parameter retrived for assigning the x coodinate of the goal location.
"""
desired_position_y = rospy.get_param('des_pos_y')
"""
Parameter retrived for assigning the y coodinate of the goal location.
"""

st="                                                                  "

flag_goal = 0
"""
Global variable for defining the current node state.
"""


def update_variables(): 
	"""
	Function that will costantly update the just mentioned paramiters and assign them to their global variable.

	"""
	global desired_position_x, desired_position_y, active_
	active_ = rospy.get_param('active')
	desired_position_x = rospy.get_param('des_pos_x')
	desired_position_y = rospy.get_param('des_pos_y')


def clbk_odom(msg): 
	"""
	CallBack to the odometry topic that will be needed to retrive the current x/y position of the robot in the enviroment.

	Args:
	 msg
	"""

	global position_
	position_ = msg.pose.pose.position
	

def done_cb(status,result):
	"""
	CallBack function for retriveing information about the status of the robot once the goal position is reached.

	Args:
	 status
	 result
	"""
	global flag_goal
	
	if status==3:
		print("\033[1;34;40m goal achived!"+st+"\033[0;37;40m \n")
		flag_goal = 1
	

def action_client_set_goal():
	"""
	Function for setting a new goal through the use of the action client.
	"""

	goal.target_pose.pose.position.x = desired_position_x
	goal.target_pose.pose.position.y = desired_position_y
	print("\033[1;33;40m START AUTONOMOUS DRIVE"+st+"\033[0;37;40m \n")
	client.send_goal(goal,done_cb)


def action_client_init():
	"""
	Function for the initialization of the action client and the goal message that will be sent to the action server through the clinet.
	"""

	global client 
	global goal 
	
	client = actionlib.SimpleActionClient('move_base',MoveBaseAction) # Initialization of the action client.
	client.wait_for_server()							# Waiting for the server to get ready.
	
	goal = MoveBaseGoal() 						# Initialization of the goal message.
	goal.target_pose.header.frame_id = "map"		# Setting up some parameters of the goal message.
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.orientation.w = 1.0
	
def my_callback_timeout(event):
	"""
	CallBack function used for setting up a timeout to the robot's current task.

	Args:
	 event
	"""

	if active_==1:
		print ("\033[1;31;40m Goal time expired\033[0;37;40m :" + str(event.current_real)+st)
		print("The robot didn't reach the desired position target within a 1min time span\n")
		rospy.set_param('active', 0)
		

def main():
	"""
	Function for managing the state of the robot. 

	"""
	
	global flag_goal
	rospy.init_node('go_to_desired_pos')				#initialization of the node.
	sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)	# Subscription to the odometry callback.
	rate = rospy.Rate(10)							# Time of the loop's sleeping rate.
	flag = 0									# Flags needed for keeping track of the current state of the driving modalities.
	flag_2 = 0
	
	action_client_init()		# Initialization of the action client.
	
	i = 0					# Variable used to print on screen the current position
	while(1):
	
		update_variables()	# Variables update at every loop cycle.
		
		# If the active_ paramter is set by the user to 1, the node will get to the active state.
		if active_==1:
			
			if flag == 1:
				action_client_set_goal()					# The new goal position will be set.
				rospy.Timer(rospy.Duration(60),my_callback_timeout) 	# The time out will start. 
				
				flag = 0
				flag_2 = 1
			
	    
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
				
		
		# Print of the current position		
		if(i%10==0):
		
			print("\033[1;37;40m coordinates \033[0;37;40m: \033[1;33;40m X:\033[0;37;40m "+ str(position_.x)+"\033[1;33;40m Y: \033[0;37;40m" + str(position_.y), end = '\r')
		i=i+1
	    		
	rate.sleep()
     

if __name__ == '__main__':
	main()
        
        
        
        
        
        
