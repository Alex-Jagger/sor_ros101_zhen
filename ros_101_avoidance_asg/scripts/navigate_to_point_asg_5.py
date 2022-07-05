#!/usr/bin/env python3
from operator import truediv
import rospy
import math, time
import utils
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import tf

DIST_AVOID_ENGAGE   = 1.0   # [m] distance to engage the steering action
AVOID_STEERING_SPEED= 60.0 * math.pi/180.0 # [rad/s] steer speed
TIME_KEEP_STEERING  = 1.0   # [s] time to keep steering even if no obstacle in sight
TIME_GO_STRAIGHT    = 2.0   # [s] time to keep going straight after a rotation to avoid the obstacle
K_YAW_TO_YAWRATE    = 1.5   # [s] Control gain from error angle to commanded yaw rate
TARGET_X = 6.0              # [m] Target X location
TARGET_Y = 0.0              # [m] Target Y location
RADIUS   = 0.1              # [m] Threshold around the target to consider it reached
STEER_ACTION_STOP= 0.2*AVOID_STEERING_SPEED  # [rad/s] maximum heading rate to allow forward motion

class NavigateAvoid():
    def __init__(self):
        
        self.range_center   = DIST_AVOID_ENGAGE*2.0 # initialize to a value higher than the thresholc
        self.cmd_fwd = 0
        self.cmd_rot = 0
        self.position_x = 0
        self.position_y = 0
        self.yaw = 0
        self.target_x = TARGET_X
        self.target_y = TARGET_Y
        self.back_to_left = False
        self.around_action_on = False
        self.around_action = 0
        self.target_in_body_frame()
        self.avoid_state = 'free'   # initialize the state machine
        self.error_angle = 0

                
        #- subscribe to the range sensor
        self.sub_center = rospy.Subscriber("/range_center", Range, self.update_range)
        rospy.loginfo("Range Subscribers set")
        #- subscribe to the source_vel coming from teleop_twist
        self.sub_center = rospy.Subscriber("/source_vel", Twist, self.update_source)
        rospy.loginfo("Source Command Subscribers set")
        #- setup the tf listener
        self.tf_listener = tf.TransformListener()
        rospy.loginfo("TF Listener set")
        
        #- we will publish the total command to the vehicle here
        self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        rospy.loginfo("Publisher set")
        #- for debug we will publish avoidance actions here
        self.pub_avoid = rospy.Publisher("/avoid_actions", Twist, queue_size=5)
        rospy.loginfo("Publisher set")
        #- initialize the messages
        self._message_cmd   = Twist()
        self._message_avoid = Twist()
        #- initialize the timer
        self._time_steer    = time.time() 
        self._time_straight = time.time() 
        

    #- callback for the tf listener
    def update_pose(self):
        (trans, rot) = self.tf_listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0))
        self.position_x = trans[0]
        self.position_y = trans[1]
        (_, _, self.yaw) = euler_from_quaternion(rot)
        self.target_in_body_frame()            

        
        #rospy.loginfo("Position: x = %.1f  y = %.1f  yaw = %.1f deg"%(self.position_x, self.position_y, self.yaw*180.0/math.pi))
        
    #- convert target in relative cylindrical cohordinate
    def target_in_body_frame(self):
        target_car_x = self.target_x - self.position_x
        target_car_y = self.target_y - self.position_y
        (self.target_range, self.target_angle) = utils.xy_to_range_angle(target_car_x, target_car_y)
        #rospy.loginfo("Target: rng = %.1f  ang = %.1f"%(self.target_range, self.target_angle*180.0/math.pi))
        
    #- calculate the feedback action for steering command
    def get_target_yaw_rate(self, k_yaw):
        self.error_angle = utils.error_angle(self.target_angle, self.yaw)
        comd_yaw_rate = k_yaw * self.error_angle
        # Limit the rate to the max
        comd_yaw_rate = max(comd_yaw_rate,-AVOID_STEERING_SPEED)
        comd_yaw_rate = min(comd_yaw_rate, AVOID_STEERING_SPEED)
        return (comd_yaw_rate)

    #- callback of the central sonar topic
    def update_range(self, message):
        angle = message.field_of_view
        self.range_center = message.range
        
    #- callback of the source commands
    def update_source(self, message):
        self.cmd_fwd = message.linear.x
        self.cmd_rot = message.angular.z    
        

    #- calculate the the target point of a corner
    def cal_tangent_point(self, car_x, car_y, corner_range, corner_yaw, side):
        corner_radius = 0.5
        target_temp_range = math.sqrt(corner_range**2 + corner_radius**2)
        addition_yaw = math.atan(corner_radius/corner_range)

        target_temp_yaw = corner_yaw + side*addition_yaw
        target_car_x = target_temp_range*math.cos(target_temp_yaw)
        target_car_y = target_temp_range*math.sin(target_temp_yaw)


        target_x = car_x + target_car_x
        target_y = car_y + target_car_y

        return(target_x, target_y)

    #- apply the control actions
    def avoid_obstacle(self):
        """
        Based on the current ranges, calculate the command
        """
        break_action   = 1.0
        steer_action   = 0.0

        #--- Get the current distance
        if (self.avoid_state == 'stop'):
            break_action = 0.0
            steer_action = 0.0
            if (self.target_range > RADIUS):
                self.avoid_state = 'free'
                
        elif (self.avoid_state == 'free'):
            break_action = 1.0
            steer_action = self.get_target_yaw_rate(K_YAW_TO_YAWRATE)
            if (self.target_range < RADIUS):
                self.avoid_state = 'stop'
            else:
                if abs(steer_action) > STEER_ACTION_STOP:
                    break_action = 0
                if self.range_center < DIST_AVOID_ENGAGE and (self.error_angle < math.pi/10):
                    self.avoid_state    = 'scan_right'
        
        elif (self.avoid_state == 'scan_right'):
            if self.around_action_on and self.around_action == 'left':
                self.avoid_state = 'scan_left'
            else:
                steer_action        = -AVOID_STEERING_SPEED/2
                break_action        = 0.0

                if self.range_center > 999:
                    self.right_corner_range = self.previous_range
                    self.right_corner_yaw = self.previous_yaw
                    self.avoid_state = "scan_left"
                else:
                    self.previous_range = self.range_center
                    self.previous_yaw = self.yaw
        
        elif (self.avoid_state == 'scan_left'):
            if self.around_action_on and self.around_action == 'right':
                self.target_x, self.target_y = self.cal_tangent_point(self.position_x, self.position_y, self.right_corner_range, self.right_corner_yaw, -1)
                self.avoid_state = 'go_target_temp'
            else: 
                steer_action        = AVOID_STEERING_SPEED/2
                break_action        = 0.0

                if not self.back_to_left:
                    if self.range_center < 999:
                        self.back_to_left = True
                else:
                    if self.range_center < 999:
                        self.previous_range = self.range_center
                        self.previous_yaw = self.yaw
                    else:
                        if self.around_action_on:
                            if self.around_action == 'left':
                                self.target_x, self.target_y = self.cal_tangent_point(self.position_x, self.position_y, self.previous_range, self.previous_yaw, 1)
                            else:
                                self.target_x, self.target_y = self.cal_tangent_point(self.position_x, self.position_y, self.right_corner_range, self.right_corner_yaw, -1)
                        else:
                            if self.right_corner_range > self.previous_range:
                                self.target_x, self.target_y = self.cal_tangent_point(self.position_x, self.position_y, self.previous_range, self.previous_yaw, 1)
                                self.around_action_on = True
                                self.around_action = 'left'
                            else:
                                self.target_x, self.target_y = self.cal_tangent_point(self.position_x, self.position_y, self.right_corner_range, self.right_corner_yaw, -1)
                                self.around_action_on = True
                                self.around_action = 'right'
                        self.avoid_state = 'go_target_temp'
                        self.back_to_left = False

        elif (self.avoid_state == 'go_target_temp'):
            break_action = 1.0
            steer_action = self.get_target_yaw_rate(K_YAW_TO_YAWRATE)
            if abs(steer_action) > STEER_ACTION_STOP:
                break_action = 0
            if self.range_center < DIST_AVOID_ENGAGE:
                self.target_x = TARGET_X
                self.target_y = TARGET_Y
                self.avoid_state    = 'free'
                self.update_pose()

        else:
            break_action = 1.0
            steer_action = 0.0
            self.avoid_state = 'free'
            
            
        if (self.cmd_fwd == 0.0):
            steer_action = 0.0
                    
        rospy.loginfo("state: %s"%(self.avoid_state))
            
        return (break_action, steer_action)
        
    def run(self):
        
        #--- Set the control rate
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            #-- Get the control action
            try:
                self.update_pose()
            except:
                continue
            break_action, steer_action = self.avoid_obstacle()
            
            #rospy.loginfo("Throttle = %3.1f    Steering = %3.1f"%(break_action, steer_action))
            
            #-- update the message
            self._message_cmd.linear.x  = break_action * self.cmd_fwd
            self._message_cmd.angular.z = steer_action + self.cmd_rot 

            self._message_avoid.linear.x  = break_action
            self._message_avoid.angular.z = steer_action
            
            #-- publish it
            self.pub_twist.publish(self._message_cmd)
            self.pub_avoid.publish(self._message_avoid)
            
            rate.sleep()        
            
if __name__ == "__main__":

    rospy.init_node('navigate_avoid')
    
    navigate_avoid     = NavigateAvoid()
    navigate_avoid.run()            
    
    
