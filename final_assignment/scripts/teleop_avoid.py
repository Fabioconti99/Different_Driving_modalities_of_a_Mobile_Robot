#!/usr/bin/env python3

"""
.. module:: teleop_avoid
 :platform: Unix
 :synopsis: Python node for robot's keyboard-input driving capapilities.

.. moduleauthor:: Fabio Conti <s4693053@studenti.unige.it>

Publishes to:
 /cmd_vel
 
The script is based on the standard ROS `teleop_twist_keyboard.py <http://wiki.ros.org/teleop_twist_keyboard>`.
This node is constantly checking which keys are pressed on a PC keyboard and based on the pressed keys, publishes twist messages on the ``/cmd_vel`` topic. Twist message defines what should be the linear and rotational speeds of a mobile robot.

I added some functions and changes to the code to merge it with the `avoidence.py` publisher and to manage the alternity of the activation state.
Just like all the other programs, I had to manage the alternity of the *idle* to the *active* state through the use of an If-statement. Since the node also includes the functionalities of the third modality, the statement will set the node to an active modality if the ``active`` param is either set to 3 or 2.
I added a new function named ``stop_motion`` to the ``PublishThread`` class. This function will make the robot stop once the driving modality gets switched. The function will set the linear and angular velocity to 0 with the `twist` message through the ``/cmd_vel`` topic.

"""

from __future__ import print_function

import threading
from sensor_msgs.msg import LaserScan
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from final_assignment.msg import Avoid

from geometry_msgs.msg import Twist
import time
from std_srvs.srv import *
import sys, select, termios, tty


# Initial message
msg = """
\033[1;37;40m
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   j    i    l
        k    

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
\033[0;37;40m
"""

ok_left = 1
"""
Variables for determining which direction is allowed.
"""
ok_right = 1
"""
Variables for determining which direction is allowed.
"""
ok_straight = 1
"""
Variables for determining which direction is allowed.
"""

moveBindings = {
        'i':(1,0,0,0),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'k':(-1,0,0,0),
    }
"""
Variables for determining which direction is allowed.
"""


# Dictionary for speed commands
speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

class PublishThread(threading.Thread):

    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

	# Function needed for stopping the robot motion.
    def stop_motion(self):
        """
        I added this new function named ``stop_motion`` to the ``PublishThread`` class. This function will make the robot stop once the driving modality gets switched. The function will set the linear and angular velocity to 0 with the ``twist`` message through the ``/cmd_vel`` topic.
        Args:
         self
        
        No returns 
        """
        twist = Twist()
        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        
        self.publisher.publish(twist)

    def run(self):
        twist = Twist()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist)


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def cb_avoidence(msg):
	"""
	The node *subscribes* to the custom topic ``custom_controller`` implemented for publishing the `Avoid. msg` message containing info about the walls surrounding the robot. The callback subscribed sets some local variables equal to the published fields of the custom message. The following uses these variables function to change some keyboards inputs to prevent the user to drive the robot into walls.
	Args:
	 msg (/custom_Controller): custom message built to retrive informtion about the position of the walls surrounding the robot.
	 
	 No Returns
	"""
	global ok_left
	global ok_right
	global ok_straight
	
	ok_right = msg.right
	ok_straight = msg.front
	ok_left = msg.left
	


# Function needed for preventing a certain command if the direction is blocked.
def new_dict(dictionary):
    """
    The function uses the ``pop`` command to directly remove some keys from the dictionary. The removal will happen accordingly to the values retrieved by the previously mentioned callback to the ``custom_controller`` topic. The values retrieved by the teleop node are relocated in the following local variables:
    
    * `ok_right`:
    	* 1 = the wall is not close to the right of the robot. The user will be able to turn right. 
    	* 0 = the wall is close to the right of the robot. The user will not be able to turn right.
    
    * `ok_left`:
    	* 1 = the wall is not close to the left of the robot. The user will be able to turn left. 
    	* 0 = the wall is close to the left of the robot. The user will not be able to turn right.
    
    * `ok_straight`:
    	* 1 = the wall is not close to the front of the robot. The user will be able to drive straight. 
    	* 0 = the wall is close to the front of the robot. The user will not be able to drive straight.
    	
    Args
     dictionary (dict): dictionary used to pop elements related to the avoidance feature.
     
    No Returns
    """
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
        






def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":

    rospy.init_node('teleop_avoid') 					# Initialization of the node.
    active_=rospy.get_param("/active")					# Assignment of the active param value to a local variable.
    rospy.Subscriber("custom_controller", Avoid, cb_avoidence)	# Subscriber to the custom topic.
    flag = 1									# Flag variable used to keep track of the current state.
    flag_2 = 0									# Flag variable used to keep track of the current state.
    
    
    # Setting up some initial parameters.
    settings = termios.tcgetattr(sys.stdin)
    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.1)
    
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    rate = rospy.Rate(5)
    pub_thread.wait_for_subscribers()
    pub_thread.update(x, y, z, th, speed, turn)
    moveBindings_temp = {}
    
    
    print(msg)				# Print of the initial message.
    print(vels(speed,turn))		# Print of the robot's state info.
    
    while not rospy.is_shutdown():
        	
        active_=rospy.get_param("/active")		# Update of the modality param value

        moveBindings_temp = moveBindings.copy()		# Coping the actual moveBindings dictionary into a temp one.
        
        
        # If the third or second modalities are active
        if active_ == 2 or active_ == 3:		
        	
            if flag_2 == 0:
            	print("\033[1;34;40m ACTIVE MODALITY "+ str(active_) +" \033[0;37;40m")
            	
            flag_2 = 1
            key = getKey(key_timeout)

            new_dict(moveBindings_temp)				# Calculating a new dict. takeing care of the messages recived by the avoidence node.
            
            
            if key in moveBindings_temp.keys():

                x = moveBindings_temp[key][0] 
                y = moveBindings_temp[key][1]
                z = moveBindings_temp[key][2]
                th = moveBindings_temp[key][3]

            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                    continue
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            pub_thread.update(x, y, z, th, speed, turn)
            flag = 1

        else:
            if flag == 1:
                pub_thread.stop_motion() 
                print("\033[1;31;40m STOP TELEOP MODALITY\033[0;37;40m")
            flag = 0
            flag_2 = 0

        rate.sleep()
            


