#!/usr/bin/env python

# Send Shadow Location to Baxter
from __future__ import division
__author__ = "Thomas Colestock & Moaed Abd"
import rospy
from std_msgs.msg import String, Float32, UInt8
import zmq
import msgpack

context = zmq.Context()
socket = context.socket(zmp.PUB)
socket.bind("tcp://*:5558")


def callback(location):
    l = location.data
    finalLocation = 4  # specify this!!!
    if (l == finalLocation):
        packed = msgpack.dumps(l)
        socket.send(packed)


def listen():
    rospy.init_node("sr_location_logic", anonymous=True)
    rospy.Subscriber("sr_movement_phase", Float32, callback)  # the SR movement script needs to update this topic (publish to) every time a movement changes.
    rospy.spin()


if __name__ == '__main__':
    try:
        listen()
    except ROSInterruptException:
        pass
