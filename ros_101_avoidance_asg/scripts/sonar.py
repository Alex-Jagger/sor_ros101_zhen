#!/usr/bin/env python3
import rospy

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range

rospy.init_node("sonar_node", anonymous=True)
location = rospy.get_param('~location')
pub_topic = "/range_"+location
sub_topic = "/scan_"+location

pub = rospy.Publisher(pub_topic, Range, queue_size=10)


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.ranges[0])
    range_msg = Range()
    range_msg.header.stamp = rospy.Time.now()
    range_msg.header.frame_id = data.header.frame_id
    range_msg.radiation_type = Range.ULTRASOUND
    range_msg.field_of_view = 0.0698132
    range_msg.min_range = 0.1
    range_msg.max_range = 30.0
    range_msg.range = min(data.ranges) 
    pub.publish(range_msg)


rospy.Subscriber(sub_topic, LaserScan, callback)

rospy.spin()
