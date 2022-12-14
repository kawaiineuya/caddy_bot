#!/usr/bin/env python2
import rospy
import math
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def get_rotation (msg):
    global roll, pitch, yaw
    orientation_q = msg.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    print math.degrees(yaw)

rospy.init_node('imu_filter_quaternion_to_euler.py')

sub = rospy.Subscriber ('/imu_e2box', Imu, get_rotation)

r = rospy.Rate(1)
while not rospy.is_shutdown():
    r.sleep()