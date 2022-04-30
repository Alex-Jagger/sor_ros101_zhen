#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

x = 0.0
y = 0.0
theta = 0.0
lin_vel = 0.0
ang_vel = 0.0

theta_int = 0.0
d_int = 0.0


k_theta_int = 30
k_theta =  20

k_d = 15 

     	
d_t = 1/500
def pose_callback(pose):
    global x, y, theta, lin_vel, ang_vel
    x = pose.x
    y = pose.y
    theta = pose.theta
    lin_vel = pose.linear_velocity
    ang_vel = pose.angular_velocity
	
rospy.init_node('turtle_guide_to_xy', log_level=rospy.DEBUG)
turtle_twist_pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
rospy.Subscriber("/turtle1/pose", Pose, pose_callback)
control = Twist()

while not rospy.is_shutdown():
	
    x_target = rospy.get_param("/x_target")
    y_target = rospy.get_param("/y_target")
        	
    delta_x = x_target - x
    delta_y = y_target - y
    delta_d = math.sqrt(delta_x**2 + delta_y**2)
    
    if (abs(delta_d > 0.01)):
        theta_target = math.atan2(delta_y, delta_x)
        delta_theta = theta_target - theta
        if (abs(delta_theta) > math.pi):
            if (delta_theta > 0) :
                delta_theta -= math.pi*2
            else:
                delta_theta += math.pi*2
        
        if (abs(delta_theta) < 0.1):
            # Displacement control
            control.linear.x = k_d*delta_d
        else:
            control.linear.x = 0.0
    
        # Attitude control
        theta_int += delta_theta*d_t
        control.angular.z = k_theta_int*theta_int + k_theta*delta_theta
        rospy.loginfo("Moving to (%s, %s)", x_target, y_target)
    else:
        control.linear.x = 0.0
        control.angular.z = 0.0
        theta_int = 0.0
        rospy.loginfo("Target Reached")
    turtle_twist_pub.publish(control)
    rospy.logdebug("(%s, %s)", x, y)
    
    
	

