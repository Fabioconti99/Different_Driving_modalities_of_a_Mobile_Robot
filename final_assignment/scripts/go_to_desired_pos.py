#!/usr/bin/env python

# libraries imports 
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalID 
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import time
import math

"""
go_to_desired_pos NODE: 

This script exploits an action client istance to establish a direct comunication with the mobile robot and set and cancel location goals. 
"""

active_ = rospy.get_param('active') 			# Parameter retrived for keeping track of the current driveing modality.
desired_position_x = rospy.get_param('des_pos_x')	# Parameter retrived for assigning the x coodinate of the goal location.
desired_position_y = rospy.get_param('des_pos_y')	# Parameter retrived for assigning the y coodinate of the goal location.

st="                                                                  "


# Function that will costantly update the just mentioned paramiters and assign them to their global variable
def update_variables(): 
	global desired_position_x, desired_position_y, active_
	active_ = rospy.get_param('active')
	desired_position_x = rospy.get_param('des_pos_x')
	desired_position_y = rospy.get_param('des_pos_y')


# CallBack to the odometry topic that will be needed to retrive the current x/y position of the robot in the enviroment.
def clbk_odom(msg): 

	global position_
	position_ = msg.pose.pose.position

# Function for setting a new goal throuh the use of the action client.
def action_client_set_goal():

	goal.target_pose.pose.position.x = desired_position_x
	goal.target_pose.pose.position.y = desired_position_y
	print("\033[1;33;40m START AUTONOMOUS DRIVE"+st+"\033[0;37;40m \n")
	client.send_goal(goal)
	
# Function for the initialization of the action client and the goal message that will be sent to the action server through the clinet.
def action_client_init():

	global client 
	global goal 
	
	client = actionlib.SimpleActionClient('move_base',MoveBaseAction) # Initialization of the action client.
	client.wait_for_server()							# Waiting for the server to get ready.
	
	goal = MoveBaseGoal() 								# Initialization of the goal message.
	goal.target_pose.header.frame_id = "map"					# Setting up some parameters of the goal message.
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.orientation.w = 1.0
	
# Call Back used for setting up a timeout to the robot's current task.
def my_callback_timeout(event):
	if active_==1:
		print ("\033[1;31;40m Goal time expired\033[0;37;40m :" + str(event.current_real)+st)
		print("The robot didn't reach the desired position target within a 1min time span\n")
		rospy.set_param('active', 0)
		

def main():

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
				action_client_set_goal()	# The new goal position will be set.
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
        
        
        
        
        
        
