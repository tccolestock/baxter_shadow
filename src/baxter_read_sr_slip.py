#!/usr/bin/env python

from __future__ import division
__author__ = "Thomas Colestock & Moaed Abd"
import rospy
from std_msgs.msg import String, Float32, UInt8
import zmq
import msgpack
import time

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://localhost:5557")
socket.setsockopt(zmq.SUBSCRIBE, b'')


def talker():
    rospy.init_node("read_sr_slip", anonymous=True)
    pub_handle = rospy.Publisher("sr_slip_logic", Float32, queue_size=10)
    time.sleep(1)
    rate_handle = rospy.Rate(500)  # hz

    while not rospy.is_shutdown():
        rec = socket.recv()
        msg = msgpack.loads(rec)
        # Conversions?
        pub_handle.publish(float(msg))
        rate_handle.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
