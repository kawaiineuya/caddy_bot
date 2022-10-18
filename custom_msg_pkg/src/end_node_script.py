#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy				# ROS 라이브러리
from custom_msg_pkg.msg import second_msg    # 서브스크라이브하는 second_msg.msg 파일 불러오기


# 퍼블리셔 노드로부터 토픽을 받아들이는 콜백 함수
def second_topic_callback(data):
    # 받은 내용(data)를 터미널에 출력
    rospy.loginfo("------")
    rospy.loginfo("Processing Time(nsec): %d", rospy.Time.now().nsecs - data.start_time.nsecs)
    rospy.loginfo("Message Sequence: %d", data.msg_seq)
    rospy.loginfo("Original | Square | Square Root: %d | %d | %d", data.original_num, data.square_num, data.sqrt_num)
    
def main():
    rospy.init_node('end_node', anonymous=False)
    rospy.Subscriber("second_topic", second_msg, second_topic_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
