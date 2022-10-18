#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy				# ROS 라이브러리
import random               # 랜덤 수를 추출하기 위한 라이브러리
import math
from custom_msg_pkg.msg import first_msg, second_msg
    # 서브스크라이브하는 first_msg.msg 파일과, 퍼블리시하는 second_msg.msg 파일 불러오기


class Calc:
    def __init__(self):
        rospy.Subscriber("/first_topic", first_msg, self.first_topic_callback)
        self.pub = rospy.Publisher("/second_topic", second_msg, queue_size=10)

        self.start_time = rospy.Time.now()
        self.msg_seq = 0
        self.original_num = 0
        self.square_num = 0
        self.sqrt_num = 0
        
    # 퍼블리셔 노드로부터 토픽을 받아들이는 콜백 함수
    def first_topic_callback(self, data):
        self.start_time = data.start_time
        self.msg_seq = data.msg_seq
        self.original_num = data.original_num
        self.square_num = math.pow(self.original_num, 2)
        self.sqrt_num = math.sqrt(self.original_num)

        # 받은 내용(data)를 터미널에 출력
        rospy.loginfo("------")
        rospy.loginfo("Message Sequence: %d", self.msg_seq)
        rospy.loginfo("Original Number: %d", self.original_num)
        rospy.loginfo("Square Number: %d", self.square_num)
        rospy.loginfo("Square Root Number: %d", self.sqrt_num)

    def second_msg_publish(self):
        msg = second_msg()	# 메시지 변수 선언

        # 메시지 내용 담기
        ## start_time: start_node가 메시지를 생성해 publish 하는 시각
        ## msg_seq: 메시지 순서(번호)
        ## original_num: 계산의 대상이 될 수
        ## square_num: original_num의 제곱
        ## sqrt_num: original_num의 제곱근
        msg.start_time = self.start_time
        msg.msg_seq = self.msg_seq
        msg.original_num = self.original_num
        msg.square_num = self.square_num
        msg.sqrt_num = self.sqrt_num

        self.pub.publish(msg)
    
def main():
    # 노드 초기화.
    rospy.init_node('middle_node', anonymous=False)
    rate = rospy.Rate(1)

    calc = Calc()
    while not rospy.is_shutdown():
        calc.second_msg_publish()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
