#!/usr/bin/env python

import rospy

import numpy as np

from baxter_shadow.srv import PdcBaseline
from sr_robot_msgs.msg import BiotacAll, Biotac


rospy.init_node("pdc_baseline_server", anonymous=True)
rate_handle = rospy.Rate(100) #hz

def server():
    s = rospy.Service("pdc_baseline", PdcBaseline, base)
    rospy.spin()


def base(req):
    i = 0
    while len(db) < 50:
        rospy.Subscriber("/rh/tactile", BiotacAll, collect)
        print(i)
        i += 1
        rate_handle.sleep()
    baseline = np.average(db)
    return baseline


def collect(data):
    p = int(data.tactiles[0].pdc)
    global db
    db.append(p)


if __name__ == '__main__':
    db = []
    server()
