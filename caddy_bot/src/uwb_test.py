#!/usr/bin/env python2
import rospy
from nav_msgs.msg import Odometry

def callback(msg):
    print (msg.twist.twist.angular.z)


def listener():
    rospy.init_node('odometry_filtered_listener')
    rospy.Subscriber("/odometry/filtered", Odometry, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()


# import math

# x = 0
# y = 1

# position_self = [0,0]
# position_B = [0.3,-0.5]
# position_C = [-0.3,-0.5]

# range_self = 1.5
# range_B = 2
# range_C = 2

# A = ( (-2*position_self[x]) + (2*position_B[x]) )
# B = ( (-2*position_self[y]) + (2*position_B[y]) )
# C = (range_self*range_self) - (range_B*range_B) - (position_self[x]*position_self[x]) + (position_B[x]*position_B[x]) - (position_self[y]*position_self[y]) + (position_B[y]*position_B[y])
# D = ( (-2*position_B[x]) + (2*position_C[x]) )
# E = ( (-2*position_B[y]) + (2*position_C[y]) )
# F = (range_B*range_B) - (range_C*range_C) - (position_B[x]*position_B[x]) + (position_C[x]*position_C[x]) - (position_B[y]*position_B[y]) + (position_C[y]*position_C[y])


# _x = (C*E-F*B) / (E*A-B*D)
# _y = (C*D-A*F) / (B*D-A*E)

# print(_x,_y)
#!/usr/bin/env python2