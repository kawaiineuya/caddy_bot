#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy				# ROS 라이브러리
import math
import serial
import binascii
import struct
from sensor_msgs.msg import NavSatFix

class Calc:
  def __init__(self):
    self.pub = rospy.Publisher("/vc_gps", NavSatFix, queue_size=10)
    self.serial_port = serial.Serial(
      port = "/dev/ttyUSB0",
      baudrate = 115200
    )
    self.count = 0
    self.aryhex = ''
    self.aryint = ''
    self.Hu_latitude = ''
    self.caddy_latitude = ''
  def hex_to_double(self, f):  
    return struct.unpack('!d', f.decode('hex'))[0]
      
  def vc_gps_msg_publish(self):
    msg = NavSatFix()
    self.data = 0
    arr = ['1','0']
    tog = 1;
    msg.latitude = self.Hu_latitude
    msg.latitude = self.caddy_latitude
    for i in range(0,32):
      self.data = self.serial_port.read()
      ary = [] 
      ary.append(hex(ord(self.data)))
      print(ary)
      self.aryint = int(ary[0], 16)
      self.aryhex = format(self.aryint,'x') + self.aryhex
      #print(aryhex)
      #print(type(aryhex)) 
      self.count = self.count + 1
      #self.serial_port.write(arr[tog].encode('utf-8'))
      #if self.serial_port.inWaiting():
      if self.count == 6:
       self.aryhex = ''
      if self.count >= 7 and self.count <= 14:  
       if self.aryint & 0x0f == 0x00:
        self.aryhex = self.aryhex + '0'
      if self.count == 14:
        self.Hu_latitude = self.aryhex
      
      if self.count == 15:
       self.aryhex = ''
      if self.count >= 16 and self.count <= 23:  
       if self.aryint & 0x0f == 0x00:
        self.aryhex = self.aryhex + '0'
      if self.count == 23:
        self.caddy_latitude = self.aryhex

    print(self.count)
    print(self.Hu_latitude)
    print(self.caddy_latitude)
    #print(ary)
    
    #self.count = self.count + 1
    if self.count == 32:
      #msg.latitude = self.hex_to_double(self.aryhex)
      #print(msg)
      print(self.hex_to_double(self.Hu_latitude))
      print(self.hex_to_double(self.caddy_latitude))
      self.count = 0
      #self.count = 0
      self.aryhex = ''
      self.aryint = ''
    self.pub.publish(msg)
    
def main():
  # 노드 초기화.
  rospy.init_node('vc_node', anonymous=False)
  rate = rospy.Rate(1)

  calc = Calc()
  while not rospy.is_shutdown():
    calc.vc_gps_msg_publish()
    rate.sleep()

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
