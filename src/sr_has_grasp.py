#!/usr/bin/env python

from __future__ import division
import rospy
from std_msgs.msg import String, Float32, UInt8
from sr_robot_msgs.msg import BiotacAll
import zmq
import msgpack

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5555")


def callback(bios):
    first_pdc = bios.tactiles[0].pdc
    thumb_pdc = bios.tactiles[4].pdc
    if (first_pdc > 2300) and (thumb_pdc > 1750):
        msg = 1
    else:
        msg = 0
    print(msg)
    packed = msgpack.dumps(msg)
    socket.send(packed)



def listen():
    rospy.init_node("sr_grip_logic", anonymous=True)
    rospy.Subscriber("rh/tactile", BiotacAll, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listen()
    except rospy.ROSInterruptException:
        pass
