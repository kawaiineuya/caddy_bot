#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float32

flag1 = False
flag2 = False

_d = 0.42
_l = 0.0
_r = 0.0

# _d = 0.42
# _l = 1.63
# _r = 1.14


def get_l (msg):
    global _l
    _l = msg.data-0.7

def get_r (msg):
    global _r
    _r = msg.data -0.5

rospy.init_node('cac_xy')
subl = rospy.Subscriber ('/floatter1', Float32, get_l)
subr = rospy.Subscriber ('/floatter2', Float32, get_r)

r = rospy.Rate(10)
while not rospy.is_shutdown():
    if ((_l !=0.0) & (_r != 0.0)):
        print(_l,_r)
        try:
            _x = (pow(_r,2) - pow(_d,2) - pow(_l,2))/(-2*_d)
            _y = math.sqrt(pow(_l,2) - pow(_x,2))
            print(_x-_d/2,_y)
        except:
            print("error")
    r.sleep()