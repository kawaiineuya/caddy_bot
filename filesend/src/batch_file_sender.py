#!/usr/bin/env python
#simple proxy to send multiple files
import rospy
import filesend.srv
import os
import std_msgs.msg

rospy.init_node("fsend_batch")
file_send_queue = []
send = rospy.ServiceProxy("send_file", filesend.srv.RequestFile)


def send_next():
    if len(file_send_queue) == 0:
        return
    trying = file_send_queue[0]
    response = send(trying)
    if response == 0:
        return
    elif response == 1:
        rospy.logwarn("Couldn't send file {} because it wasn't found on disk".format(trying.local_file))
        file_send_queue.pop(0)
        send_next()
    else:
        while response == 2:
            rospy.sleep(10)
            response = send(trying)


def on_sent(msg):
    if len(file_send_queue) > 0 and msg.data != "":
        rospy.sleep(2)
        file_send_queue.pop(0)
        send_next()
    elif len(file_send_queue) > 0 and msg.data == "":
        send_next()
    elif len(file_send_queue) == 0 and msg.data != "":
        queue_done.publish()
    else:
        pass


sent_subscriber = rospy.Subscriber("file_saved", std_msgs.msg.String, queue_size=5, callback=on_sent)
queue_done = rospy.Publisher("queue_done", std_msgs.msg.Empty, queue_size=4)


def add_more(msg):
    f = False
    if len(file_send_queue) == 0:
        f = True
    for s in msg.local_files:
        file_send_queue.append(
            filesend.srv.RequestFileRequest(local_file=s, folder=msg.folder)
        )
    if f:
        send_next()
    return filesend.srv.RequestManyFilesResponse()

_add_more = rospy.Service("add_request_queue", filesend.srv.RequestManyFiles, add_more)
rospy.spin()