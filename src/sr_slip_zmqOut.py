#!/usr/bin/env python

# Send Shadow Slip to Baxter

from __future__ import division
__author__ = "Thomas Colestock & Moaed Abd"
import rospy
from std_msgs.msg import String, Float32, UInt8
import zmq
import msgpack

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5557")


def callback(slip):
    s = slip.data

    packed = msgpack.dumps(s)
    socket.send(packed)


def listen():
    rospy.init_node("sr_slip_logic", anonymous=True)
    rospy.Subscriber("Mat_Sim_Slip_FF", Float32, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listen()
    except ROSInterruptException:
        pass
