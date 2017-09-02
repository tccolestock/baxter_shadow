#!/usr/bin/env python

from __future__ import division
import time

import rospy

from std_msgs.msg import String, Float32, UInt8

from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander

rospy.init_node("standtest", anonymous=True)

hand_commander = SrHandCommander()
arm_commander = SrArmCommander()
rate_handle = rospy.Rate(100) #hz

time.sleep(1)

# Hand positions

hand_start = {
        'rh_FFJ1': -0.013387468694274651,   'rh_FFJ2': 0.10550124582950798,
        'rh_FFJ3': -0.07913645703418956,    'rh_FFJ4': -0.020790969983510318,
        'rh_THJ4': 0.8987090669167258,      'rh_THJ5': -1.0529838245665772,
        'rh_THJ1': 0.36613957472880915,     'rh_THJ2': -0.3099264451304632,
        'rh_THJ3': 0.04339213288734181,     'rh_LFJ2': 0.11856120196799154,
        'rh_LFJ3': -0.14247924347682977,    'rh_LFJ1': 0.020856552138779016,
        'rh_LFJ4': 0.006156109478006114,    'rh_LFJ5': 0.000035368858695598477,
        'rh_RFJ4': -0.017502072148899307,   'rh_RFJ1': 0.04862574836081379,
        'rh_RFJ2': 0.23106641618794493,     'rh_RFJ3': -0.040169677117662395,
        'rh_MFJ1': 0.0061621824517631985,   'rh_MFJ3': -0.03814186780706377,
        'rh_MFJ2': 0.28535536916148746,     'rh_MFJ4': 0.005735133335643892,
        }

hand_close = {
        'rh_FFJ1': 0.5366228138727492,      'rh_FFJ2': 1.3707472622836295,
        'rh_FFJ3': 0.6104890181588297,      'rh_FFJ4': -0.1693188654196813,
        'rh_THJ4': 1.1494816044032174,      'rh_THJ5': -0.25236240595266746,
        'rh_THJ1': 1.0564478227578378,      'rh_THJ2': 0.5591902548242037,
        'rh_THJ3': 0.3010860128238289,      'rh_LFJ2': 1.1510589476677358,
        'rh_LFJ3': 0.3496450123403709,      'rh_LFJ1': 0.2812655031286765,
        'rh_LFJ4': 0.0007317935784767475,   'rh_LFJ5': 0.038378063907728126,
        'rh_RFJ4': -0.030822436892029084,   'rh_RFJ1': 0.2252787835450361,
        'rh_RFJ2': 1.1696882711839942,      'rh_RFJ3': 0.6358242015720096,
        'rh_MFJ1': 0.18990725919524606,     'rh_MFJ3': 0.6792600589796994,
        'rh_MFJ2': 1.3251573950327318,      'rh_MFJ4': -0.007377111269187729,
        }

# Arm positions

pickup = {
    'ra_shoulder_pan_joint': -0.64500271116839808,
    'ra_shoulder_lift_joint': -1.2458885847674769,
    'ra_elbow_joint': 2.7204307918548584,
    'ra_wrist_1_joint': 1.6558868989944458,
    'ra_wrist_2_joint': -1.640662972127096,
    'ra_wrist_3_joint': -1.5906219450580042,
    'rh_WRJ1': -0.0851542173817335,
    'rh_WRJ2': -0.02776222781084232
    }

exit_pickup = {
    'ra_shoulder_pan_joint': -0.64500271116839808,
    'ra_shoulder_lift_joint': -1.6580899397479456,
    'ra_elbow_joint': 2.652906656265259,
    'ra_wrist_1_joint': 2.129228115081787,
    'ra_wrist_2_joint': -1.640674893056051,
    'ra_wrist_3_joint': -1.5906219450580042,
    'rh_WRJ1': -0.07590067860170616,
    'rh_WRJ2': -0.02776222781084232
    }

if __name__ == '__main__':

    joint_goals = hand_start
    hand_commander.move_to_joint_value_target_unsafe(joint_goals, 3.0, True)

    joint_goals = pickup
    arm_commander.move_to_joint_value_target_unsafe(joint_goals,5,True)
    #
    # joint_goals = hand_close
    # hand_commander.move_to_joint_value_target_unsafe(joint_goals, 5, True)
    #
    #
    # joint_goals = exit_pickup
    # arm_commander.move_to_joint_value_target_unsafe(joint_goals,3,True)
