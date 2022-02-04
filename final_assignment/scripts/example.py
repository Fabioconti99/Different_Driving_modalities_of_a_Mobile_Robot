#! /usr/bin/env python3

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseActionGoal

from actionlib_msgs.msg import GoalID 

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import time
import math

goal_msg=MoveBaseActionGoal()
goal_cancel=GoalID()
position_=Point()
my_timer = 0
goal_msg.goal.target_pose.header.frame_id = 'map'
goal_msg.goal.target_pose.pose.orientation.w = 1
active_ = rospy.get_param('active')
desired_position_x = rospy.get_param('des_pos_x')
desired_position_y = rospy.get_param('des_pos_y')

def set_goal(x, y):
	goal_msg.goal.target_pose.pose.position.x = x
	goal_msg.goal.target_pose.pose.position.y = y
	pub_goal.publish(goal_msg)

def clbk_odom(msg):

	global position_
	position_ = msg.pose.pose.position

def update_variables():
	global desired_position_x, desired_position_y, active_
	active_ = rospy.get_param('active')
	desired_position_x = rospy.get_param('des_pos_x')
	desired_position_y = rospy.get_param('des_pos_y')

def my_callback_timeout(event):
	if active_==1:
		print ("Timer called at " + str(event.current_real))
		print("Position not reached, target canceled\n")
		rospy.set_param('active', 0)
		



def main():

	global pub_vel 
	global pub_goal

	flag = 0
	flag_2 = 0
	
	rospy.init_node('go_to_desired_pos')
	pub_goal = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)
	pub_cancel_goal = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
	pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

	pub_goal.publish(goal_msg)
	sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
	rate = rospy.Rate(10)

	
	i=0
	while (1):
		
		update_variables()


		if(i%5==0):
			print("X: " + str(position_.x))
			print("Y: " + str(position_.y))
		i=i+1

		if active_==1:
		
			if flag_2 == 0:
				print("\033[1;32;40m ACTIVE MODALITY 1 \033[0;37;40m")
			flag_2 = 1
						
			if flag == 1:
				rospy.Timer(rospy.Duration(60),my_callback_timeout)
				set_goal(desired_position_x, desired_position_y)
				flag = 0

			if abs(desired_position_x-position_.x) <= 0.4 and abs(desired_position_y-position_.y) <= 0.4:
				print("Target reached, IDLE\n")
				rospy.set_param('active', 0)

		else:
			if flag == 0:
				print("\033[1;31;40m STOP MODALITY 1 \033[0;37;40m \n")
				pub_cancel_goal.publish(goal_cancel)
				flag = 1
				flag_2 = 0

		rate.sleep()

	


if __name__ == '__main__':
    main()




	




	
