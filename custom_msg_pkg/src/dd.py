#!/usr/bin/env python
#-*- coding:utf-8 -*-

from itertools import count
from re import S
import rospy				# ROS 라이브러리
import math
import serial
import binascii
import struct
from sensor_msgs.msg import NavSatFix

class Calc:
  def __init__(self):
    # self.pub1 = rospy.Publisher("/vc_gps_cart", NavSatFix, queue_size=10)
    self.pub2 = rospy.Publisher("/vc_gps_hu", NavSatFix, queue_size=10)
    self.serial_port = serial.Serial(
      port = "/dev/ttyUSB1",
      baudrate = 115200
    )
    self.count = 0
    self.aryhex = ''
    self.aryint = ''
    self.Hu_latitude = ''
    self.Hu_Longitude = ''
    self.caddy_latitude = ''
    self.caddy_Longitude = ''
  def hex_to_double(self, f):  
    # print(f)
    # print(type(f))
    return struct.unpack('!d', f.decode('hex'))[0]
      
  def vc_gps_msg_publish(self):
    msg1 = NavSatFix()
    msg2 = NavSatFix()
    self.data = 0
    arr = ['1','0']
    tog = 1
    for i in range(0,64):
      self.data = self.serial_port.read()
      # print(self.data)
      # print(len(hex(ord(self.data))))
      if(len(hex(ord(self.data))) < 4):
        temp = hex(ord(self.data)) + "0"
      
        # print("asdasd")
        # temp = hex(ord(self.data))
        # tem = temp.split('x')
        # temp = tem[0] + "x"+"0"+tem[1]
        # print(temp)
        ary = [] 
        ary.append(temp)
      else:
        ary = [] 
        ary.append(hex(ord(self.data)))
      # ary = [] 
      # ary.append(hex(ord(self.data)))
      # print(ary)
      self.aryint = int(ary[0], 16)
      self.aryhex = format(self.aryint,'x') + self.aryhex
      if self.aryint & 0xff == 0x00:
        self.aryhex = self.aryhex + "0"
      # print(aryhex)
      # print(type(aryhex)) 
      self.count = self.count + 1
      #self.serial_port.write(arr[tog].encode('utf-8'))
      #if self.serial_port.inWaiting():
      if self.count == 6:
       self.aryhex = ''
      if self.count == 14:  
       if self.aryint & 0x0f == 0x00:
        self.aryhex = self.aryhex
      if self.count == 14:
        self.Hu_latitude = self.aryhex 
        # print(self.aryhex)    
      if self.count == 14:
       self.aryhex = ''
      if self.count >= 16 and self.count <= 24:  
       if self.aryint & 0x0f == 0x00:
        self.aryhex = self.aryhex
      if self.count == 22:
        self.Hu_Longitude = self.aryhex
      
      if self.count == 38:
       self.aryhex = ''
      if self.count >= 39 and self.count <= 46:  
       if self.aryint & 0x0f == 0x00:
        self.aryhex = self.aryhex
      if self.count == 46:
        self.caddy_latitude = self.aryhex     
      if self.count == 46:
       self.aryhex = ''
      if self.count >= 48 and self.count <= 55:  
       if self.aryint & 0x0f == 0x00:
        self.aryhex = self.aryhex
      if self.count == 54:
        self.caddy_Longitude = self.aryhex

      # print(self.count)
      if self.count >= 64:
        #msg.latitude = self.hex_to_double(self.aryhex)
        #print(msg)
        #print(msg1)
        # print(self.Hu_latitude)
        self.Hu_latitude = self.hex_to_double(self.Hu_latitude)
        self.Hu_Longitude = self.hex_to_double(self.Hu_Longitude)
        self.caddy_latitude = self.hex_to_double(self.caddy_latitude)
        self.caddy_Longitude = self.hex_to_double(self.caddy_Longitude)
        
        self.count = 0
        self.aryhex = ''
        self.aryint = ''
        # msg1.latitude = self.Hu_latitude
        # msg1.longitude = self.Hu_Longitude
        # msg1.header.frame_id = "vc_gps_cart_link"
        msg2.latitude = self.caddy_latitude
        msg2.longitude = self.caddy_Longitude
        # print(self.caddy_latitude)
        msg2.header.frame_id = "vc_gps_hu_link"
        # self.pub1.publish(msg1)
        self.pub2.publish(msg2)
        self.serial_port.reset_input_buffer()

      # print(self.count)
    
    
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
