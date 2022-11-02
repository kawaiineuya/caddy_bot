#!/usr/bin/env python
import math
import os

import rospy
import filesend.msg
import filesend.srv
import std_msgs.msg
import binascii
import pylzma
import Queue
from cStringIO import StringIO

file_queue = Queue.Queue()
target_chunks = 320  # amount of chunks minimum (increase for smaller messages)
sending = False
starting = False

rospy.init_node("fsend_sender")
pub_data = rospy.Publisher("file_data", filesend.msg.FileChunk, queue_size=30)
pub_announce = rospy.Publisher("file_announce", filesend.msg.FileBegin, queue_size=5)
pub_busy = rospy.Publisher("file_send_busy", std_msgs.msg.Bool, queue_size=5, latch=True)


def CHUNK_SIZE(for_):
    return max(2048, min((2**int(math.log(int(for_/target_chunks)+4, 2))), 4194304))


def populate_queue(f_obj):
    global file_queue
    file_queue = Queue.Queue()
    cfile = StringIO(pylzma.compress(f_obj.read(), eos=1))
    cfile.seek(0, 2)
    cfile_size = cfile.tell()
    cfile.seek(0, 0)
    c_size = CHUNK_SIZE(cfile_size)
    while abs(cfile_size - cfile.tell()) / c_size > 1:
        rospy.loginfo("Compressing data packet")
        file_queue.put(cfile.read(c_size))
    file_queue.put(cfile.read())


def request_file(msg):
    global file_queue, sending, starting
    if sending:
        return filesend.srv.RequestFileResponse(status=2)
    else:
        try:
            with open(msg.local_file, "rb") as f:
                rospy.loginfo("Compressing file!")
                populate_queue(f)
                sending = True
        except IOError:
            file_queue.empty()
            return filesend.srv.RequestFileResponse(status=1)
        pub_announce.publish(ending=False, folder=msg.folder, filename=os.path.split(msg.local_file)[1])
        rospy.sleep(1)
        starting = True
        rospy.loginfo("Sending file!")
        pub_busy.publish(True)
        return filesend.srv.RequestFileResponse(status=0)


send_serv = rospy.Service("send_file", filesend.srv.RequestFile, request_file)

while not rospy.is_shutdown():
    if not file_queue.empty() and sending:
        rospy.loginfo("Sending file, remaining packets {}: ".format(file_queue.qsize()))
        rospy.sleep(0.06)
        d = file_queue.get()
        starting = False
        crc32 = binascii.crc32(d)
        pub_data.publish(data=d, crc32=crc32)
    elif sending and not starting:
        rospy.loginfo("Sent file!")
        sending = False
        pub_announce.publish(ending=True)
        pub_busy.publish(False)
    else:
        pass
