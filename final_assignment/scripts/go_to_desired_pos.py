#!/usr/bin/env python

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


active_ = rospy.get_param('active')
desired_position_x = rospy.get_param('des_pos_x')
desired_position_y = rospy.get_param('des_pos_y')



def update_variables():
	global desired_position_x, desired_position_y, active_
	active_ = rospy.get_param('active')
	desired_position_x = rospy.get_param('des_pos_x')
	desired_position_y = rospy.get_param('des_pos_y')


def clbk_odom(msg):

	global position_
	position_ = msg.pose.pose.position

def action_client_set_goal():

	goal.target_pose.pose.position.x = desired_position_x
	goal.target_pose.pose.position.y = desired_position_y
	print("\033[1;32;40m Start autonomous drive \033[0;37;40m \n")
	client.send_goal(goal)
	
	
def action_client_init():

	global client 
	global goal 
	
	client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
	client.wait_for_server()
	
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.orientation.w = 1.0
	
		

def main():

	rospy.init_node('go_to_desired_pos')
	sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
	rate = rospy.Rate(10)
	flag = 0
	flag_2 = 0
	
	action_client_init()
	
	i = 0
	while(1):
		update_variables()
		
		
		
		if active_==1:
			
			if flag == 1:
				action_client_set_goal()
				flag = 0
				flag_2 = 1
	    
	    
		else:
			if flag == 0 and flag_2==0:
				#state with no goal set
				
				print("\033[1;31;40m STOP MODALITY 1 \033[0;37;40m \n\n\n")
				flag = 1
				
			if flag == 0 and flag_2==1:
				#state with goal already set
				
				print("\033[1;31;40m GOAL CANCELED \033[0;37;40m \n\n\n")
				client.cancel_goal()
				flag = 1
				flag_2 = 0
				
		if(i%10==0):
			print("coordinates: ", end = '\r')
			print("X: " + str(position_.x), end = '\r')
			print("Y: " + str(position_.y), end = '\r')
		i=i+1
	    		
	rate.sleep()
     

if __name__ == '__main__':
	main()
        
        
        
        
        
        
