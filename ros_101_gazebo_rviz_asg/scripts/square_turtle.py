#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time
import math

rospy.init_node('square_turtle')
publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

msg = Twist()

while not rospy.is_shutdown():
	# Move forward
	start_time = time.time()
	while (time.time() - start_time) < 1:
		msg.linear.x = 2
		msg.angular.z = 0.0
		publisher.publish(msg)
	msg.linear.x = 0.0
	publisher.publish(msg)
	
	# Turn right
	start_time = time.time()
	while (time.time() - start_time) < 1:
		msg.linear.x = 0.0
		msg.angular.z = -math.pi/2.0
		publisher.publish(msg)
	msg.angular.z = 0.0
	publisher.publish(msg)	
