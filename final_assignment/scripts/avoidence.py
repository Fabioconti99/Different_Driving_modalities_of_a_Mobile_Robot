#!/usr/bin/env python3

# Imports
from __future__ import print_function

from sensor_msgs.msg import LaserScan
import rospy
from final_assignment.msg import Avoid #custom import


ok_left = 1
ok_right = 1
ok_straight = 1

# Call back function needed for checking if any wall is close to the robot and it what direction the wall is.
def cb_avoid(msg):

	global ok_left
	global ok_right
	global ok_straight
	
	active_=rospy.get_param("/active")		# Assignment of the active param value to a local variable.
	
	
	if active_ == 3:
		
		right = min(msg.ranges[0:143])	# right checking laser span.
		front = min(msg.ranges[288:431])	# front checking laser span.
		left = min(msg.ranges[576:719])	# left checking laser span.
		
								# If the robot is close to the right of the robot.
		if right < 1.0:
			ok_right = 0
		else:
			ok_right = 1
								# If the robot is close to the front of the robot.
		if front < 1.0:
			ok_straight = 0
		else:
			ok_straight = 1
								# If the robot is close to the left of the robot.
		if left < 1.0:
			ok_left = 0
		else:
			ok_left = 1
								# Let all the direction good to go if the modality 3
								# is turned off.
	else: 
		ok_right = 1
		ok_straight = 1
		ok_left = 1
		

def main():

	global ok_left
	global ok_right
	global ok_straight
	
	pub = rospy.Publisher('custom_controller', Avoid, queue_size=10)	# Publisher.
	rospy.init_node('avoidence') 								# Initialization of the node.
	sub = rospy.Subscriber('/scan', LaserScan, cb_avoid)				# Sub to the '/scan' topic.
	rate = rospy.Rate(5) 									#10hz
	
	pub_msg = Avoid()
	
    
	while not rospy.is_shutdown():
	
		pub_msg.left = ok_left		# Assigning the messages fields
		pub_msg.right = ok_right	# Assigning the messages fields
		pub_msg.front = ok_straight	# Assigning the messages fields
		
		pub.publish(pub_msg)		# publishing the messages fields
		
		rate.sleep()				# 10hz delay.

if __name__=="__main__":
	main()
		
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        




