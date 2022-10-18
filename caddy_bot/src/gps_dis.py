#!/usr/bin/env python2
import rospy
import math
from sensor_msgs.msg import NavSatFix

class get_gps:
    def __init__(self):
        self.sub_caddy = rospy.Subscriber ('/vc_gps_caddy', NavSatFix, self.get_vc_gps_caddy)
        self.sub_hu = rospy.Subscriber ('/vc_gps_hu', NavSatFix, self.get_vc_gps_hu)
        self.caddy_lat = 0.0
        self.caddy_lon = 0.0
        self.hu_lat = 0.0
        self.hu_lon = 0.0

    def get_vc_gps_caddy (self,msg):
        self.caddy_lat = msg.latitude
        self.caddy_lon = msg.longitude
        self.point2d(self.caddy_lat,self.caddy_lon,self.hu_lat,self.hu_lon)

    def get_vc_gps_hu (self,msg):
        self.hu_lat = msg.latitude
        self.hu_lon = msg.longitude
    
    def point2d(self, caddy_lat,caddy_lon,hu_lat,hu_lon):
        a = caddy_lat - hu_lat
        b = caddy_lon - hu_lon
        print(self.caddy_lat,self.caddy_lon,self.hu_lat,self.hu_lon)
        print(100000 * (math.sqrt((a * a) + (b * b))))
        


temp = get_gps()
rospy.init_node('gps_copy')

r = rospy.Rate(10)
while not rospy.is_shutdown():
    r.sleep()