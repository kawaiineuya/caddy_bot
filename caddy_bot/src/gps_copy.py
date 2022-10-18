#!/usr/bin/env python2
import rospy
import math
from sensor_msgs.msg import NavSatFix

def get_gps (msg):
    msg1 = NavSatFix()
    msg1.header.stamp = rospy.get_rostime()
    msg1.header.frame_id = "gps_copy_link"
    msg1.latitude = msg.latitude
    msg1.longitude = msg.longitude
    msg1.altitude = msg.altitude
    # msg1.position_covariance = msg.position_covariance
    # msg1.position_covariance_type = msg.position_covariance_type
    msg1.status.service = msg.status.service
    msg1.status.status = msg1.status.status
    pub.publish(msg1)

rospy.init_node('gps_copy')

sub = rospy.Subscriber ('/ublox_gps/fix', NavSatFix, get_gps)
pub = rospy.Publisher("/ublox_gps_copy", NavSatFix, queue_size=10)

r = rospy.Rate(10)
while not rospy.is_shutdown():
    r.sleep()