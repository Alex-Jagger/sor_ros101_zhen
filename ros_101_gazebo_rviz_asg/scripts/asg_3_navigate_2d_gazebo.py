#!/usr/bin/env python
from multiprocessing.connection import Listener
import rospy
import tf
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import math

x = 0.0
y = 0.0
theta = 0.0
# lin_vel = 0.0
# ang_vel = 0.0

x_target = 0
y_target = 0
theta_final_target = 0

theta_int = 0.0
d_int = 0.0


k_theta_int = 0
k_theta =  5

k_d = 1
	
d_t = 1/500

# def pose_callback(pose):
#     global x, y, theta, lin_vel, ang_vel
#     x = pose.x
#     y = pose.y
#     theta = pose.theta
#     lin_vel = pose.linear_velocity
#     ang_vel = pose.angular_velocity

def goal_callback(pose_stamped):
    global x_target, y_target, theta_final_target
    x_target= pose_stamped.pose.position.x
    y_target= pose_stamped.pose.position.y
    q_x= pose_stamped.pose.orientation.x
    q_y= pose_stamped.pose.orientation.y
    q_z= pose_stamped.pose.orientation.z
    q_w= pose_stamped.pose.orientation.w
    (_, _, theta_final_target) = euler_from_quaternion([q_x, q_y, q_z, q_w])
    print(f"x_target: {x_target: .2f}\t y_target: {y_target: .2f}\t Yaw: {theta_final_target: .2f}")

rospy.init_node('turtle_guide_to_xy')
listener = tf.TransformListener()

turtle_twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
# rospy.Subscriber("/turtle1/pose", Pose, pose_callback)
rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_callback)
control = Twist()

while not rospy.is_shutdown():
	
    try:
        (trans, rot) = listener.lookupTransform(
            "/odom", "/base_footprint", rospy.Time(0)
        )
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue

    x = trans[0]
    y = trans[1]
    (q_x, q_y, q_z, q_w) = rot
    (_, _, theta) = euler_from_quaternion([q_x, q_y, q_z, q_w])

    # print(f"X: {x: .2f}\t Y: {y: .2f}\t Yaw: {theta: .2f}")
    # x_target = rospy.get_param("/x_target")           
    # y_target = rospy.get_param("/y_target")
        	
    delta_x = x_target - x
    delta_y = y_target - y
    delta_d = math.sqrt(delta_x**2 + delta_y**2)
    delta_theta_final = theta_final_target - theta

    if (abs(delta_d > 0.1)):
        theta_target = math.atan2(delta_y, delta_x)
        delta_theta = theta_target - theta
        if (abs(delta_theta) > math.pi):
            if (delta_theta > 0) :
                delta_theta -= math.pi*2
            else:
                delta_theta += math.pi*2
        
        if (abs(delta_theta) < 1):
            # Displacement control
            control.linear.x = k_d*delta_d
        else:
            control.linear.x = 0.0
    
        # Attitude control
        theta_int += delta_theta*d_t
        control.angular.z = k_theta_int*theta_int + k_theta*delta_theta
        # rospy.loginfo("Moving to (%s, %s)", x_target, y_target)
    elif (abs(delta_theta_final) > 0.1):
        control.angular.z = k_theta*delta_theta_final
    else:
        control.linear.x = 0.0
        control.angular.z = 0.0
        theta_int = 0.0
        # rospy.loginfo("Target Reached")
    turtle_twist_pub.publish(control)
    # rospy.logdebug("(%s, %s)", x, y)
    
    
	

