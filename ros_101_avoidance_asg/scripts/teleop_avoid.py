#!/usr/bin/env python
import rospy
import math, time
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

DIST_STEER_ENGAGE   = 2.0
DIST_BREAK          = 2.5

K_FRONT_DIST_TO_SPEED   = 1.0
K_LAT_DIST_TO_STEER     = 2.0

TIME_KEEP_STEERING      = 5.5

def saturate(value, min, max):
    if value <= min: return(min)
    elif value >= max: return(max)
    else: return(value)

class ObstAvoid():
    def __init__(self):
        
        self.range_center   = 3
        
        self.sub_center = rospy.Subscriber("/range", Range, self.update_range)
        rospy.loginfo("Range Subscribers set")
        
        self.sub_center = rospy.Subscriber("/source_vel", Twist, self.update_source)
        rospy.loginfo("Source Command Subscribers set")
        
        self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        rospy.loginfo("Publisher set")
        
        self.pub_avoid = rospy.Publisher("/avoid_actions", Twist, queue_size=5)
        rospy.loginfo("Publisher set")
        
        self._message_cmd   = Twist()
        self._message_avoid = Twist()
        
        self._time_steer      = 0
        self._steer_prev      = 0
        
        self.cmd_fwd = 0
        self.cmd_rot = 0;
        
    def update_range(self, message):
        angle = message.field_of_view
        self.range_center = message.range
        
    def update_source(self, message):
        self.cmd_fwd = message.linear.x
        self.cmd_rot = message.angular.z    
        #print("linear: %.2f",message.linear.x)

    def get_control_action(self):
        """
        Based on the current ranges, calculate the command
        """
        
        break_action   = 1.0
        steer_action   = 0.0
        
        #--- Get the minimum distance
        range   = self.range_center
        # print "%.2f    %.2f  %.2f  %.2f"%(range,  self.range_left, self.range_center, self.range_right)
        
        if self.range_center < DIST_STEER_ENGAGE:
            #--- Start applying the break
            # break_action   = (range - DIST_BREAK)/(DIST_STEER_ENGAGE - DIST_BREAK) 
            adim_dist      = range/DIST_STEER_ENGAGE

            if range < DIST_BREAK and self.cmd_fwd > 0:
                break_action   =  K_FRONT_DIST_TO_SPEED*(range/DIST_BREAK)
                break_action   = saturate(break_action, 0, 1)
                rospy.loginfo("Engaging break %.1f"%break_action)
            
            #--- Apply steering, proportional to how close is the object
            steer_action       = K_LAT_DIST_TO_STEER*(1.0 - adim_dist)
            steer_action       = self.get_signed_steer(steer_action)
            
            steer_action   = saturate(steer_action, -1.5, 1.5)
            rospy.loginfo("Steering command %.2f"%steer_action)
            
        return (break_action, steer_action)
        
    def get_signed_steer(self, steer_action):
    
        if time.time() > self._time_steer + TIME_KEEP_STEERING:
            self._time_steer  = time.time()
            self._steer_prev = steer_action
        else:
            steer_action = self._steer_prev
            
        return (steer_action)
        
    def run(self):
        
        #--- Set the control rate
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            #-- Get the control action
            break_action, steer_action = self.get_control_action()
            
            # rospy.loginfo("Throttle = %3.1f    Steering = %3.1f"%(break_action, steer_action))
            
            #-- update the message
            self._message_cmd.linear.x  = break_action * self.cmd_fwd
            self._message_cmd.angular.z = steer_action + self.cmd_rot

            self._message_avoid.linear.x  = break_action
            self._message_avoid.angular.z = steer_action
            
            print ("cmd_fwd: %.1f  break_action: %.2f  out: %.1f "%(self.cmd_fwd,break_action,self._message_cmd.linear.x))
            
            #-- publish it
            self.pub_twist.publish(self._message_cmd)
            self.pub_avoid.publish(self._message_avoid)
            
            rate.sleep()        
            
if __name__ == "__main__":

    rospy.init_node('obstacle_avoid')
    
    obst_avoid     = ObstAvoid()
    obst_avoid.run()            
