#!/usr/bin/env python

"""
Collects the grip state value of the SR Hand from the ZMQ socket, and publishes it onto a ROS topic on Baxter.
 This script is supposed to run on Baxter's roscore.
"""

from __future__ import division
__author__ = "Thomas Colestock"

import time

import rospy
import zmq
import msgpack

from std_msgs.msg import String, Float32, UInt8

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://localhost:5556")
socket.setsockopt(zmq.SUBSCRIBE, b'')


def talker():
    rospy.init_node("read_sr_grip_logic", anonymous=True)
    pub_handle = rospy.Publisher("sr_grip_logic", UInt8, queue_size=0)
    time.sleep(1)
    rate_handle = rospy.Rate(500)  # hz
    while not rospy.is_shutdown():
        recvd = socket.recv()
        msg = msgpack.loads(recvd)
        logic = int(msg)
        print(msg)
        pub_handle.publish(logic)
        rate_handle.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
