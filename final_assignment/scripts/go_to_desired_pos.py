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
	

def main():

	rospy.init_node('go_to_desired_pos')
	sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
	rate = rospy.Rate(10)
	flag = 0
    
	while(1):
		update_variables()
		
		#if(i%5==0):
			#print("X: " + str(position_.x))
			#print("Y: " + str(position_.y))
			#i=i+1
		
		if active_==1:
    	
	    
	    	
			client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
			client.wait_for_server()
	    
			goal = MoveBaseGoal()
			goal.target_pose.header.frame_id = "map"
			goal.target_pose.header.stamp = rospy.Time.now()
			goal.target_pose.pose.position.x = desired_position_x
			goal.target_pose.pose.position.y = desired_position_y
			goal.target_pose.pose.orientation.w = 1.0
	    
			client.send_goal(goal)
			wait = client.wait_for_result(timeout = rospy.Duration(60.0))
	    
			if not wait:
				print("Action server not available!")
				client.cancelGoal(goal)
				rospy.set_param('active', 0)
			else:
				client.get_result()
				rospy.set_param('active', 0)
	    
			flag = 0
	    
		else:
			if flag == 0:
				print("\033[1;31;40m STOP MODALITY 1 \033[0;37;40m \n")
				flag = 1
	    		
	rate.sleep()
     

if __name__ == '__main__':
	main()
        
        
        
        
        
        
