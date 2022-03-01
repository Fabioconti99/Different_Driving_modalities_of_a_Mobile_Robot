#!/usr/bin/env python3

# Imports
from __future__ import print_function

from sensor_msgs.msg import LaserScan
import rospy
from final_assignment.msg import Avoid #custom import


"""
.. module:: avoidence
 :platform: Unix
 :synopsys: Python node for robot's avoidence.

.. moduleauthor:: Fabio Conti <s4693053@studenti.unige.it>

Subscribes to:
 /laser_scan
 
Publishes to:
 /avoid
 
This node aims to activate a security feature for driving with the teleop_key modality. Thanks to the *subscription* to the ``/laser_scan`` topic, the node will be able to get info about the robot's surroundings. The subscription to the topic will give back the ``ranges[0,720]`` array to the subscribed callback. This data structure contains the distance values between the robot and the surrounding walls for a span of 180º degrees in front of the robot. The array simulates the info that a set of lasers would retrieve in an actual environment.
The node will later elaborate the data acquired to publish it on the ``custom_controller`` custom topic through the ``Avoid.msg`` custom message.

"""


ok_left = 1
ok_right = 1
ok_straight = 1

# Call back function needed for checking if any wall is close to the robot and it what direction the wall is.
def cb_avoid(msg):
	"""
	Callback function used to acquire and manage the data from the `/lase_scan` subscription. Once the callback retrieves the `ranges[]` array, the following 3 sub-ranges divide the data structure  as follows:
	* From 0 to 143: which represents the right side of the scanned area.
	* From 288 to 431: which represents the front side of the scanned area.
	* From 576 to 719: which represents the left side of the scanned area.
	
	Args:
	 msg
	"""

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
		
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        




