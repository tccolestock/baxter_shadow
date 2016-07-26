#!/usr/bin/env python

from __future__ import division
import rospy
from std_msgs.msg import String, Float32, UInt8
import zmq
import msgpack
import time

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://localhost:5555")
socket.setsockopt(zmq.SUBSCRIBE, b'')


def talker():
    rospy.init_node("read_sr_grip_logic", anonymous=True)
    pub_handle = rospy.Publisher("sr_grip_logic", UInt8, queue_size=10)
    time.sleep(1)
    rate_handle = rospy.Rate(500) #hz
    while not rospy.is_shutdown():
        recvd = socket.recv()
        msg = msgpack.loads(recvd)
        logic = UInt8()
        logic = int(msg)
        print(msg)
        pub_handle.publish(logic)
        rate_handle.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
