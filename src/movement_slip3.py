#!/usr/bin/env python

# !!!  UPDATE THE DESCRIPTION !!! ###

# Psuedo code:
# This script will allow the shadow hand & UR10 to pick up a "bottle" and then
# deliver it to the Baxter robot. The Biotac sensors will serve as the feedback
# to the SH and to the Baxter.
#
#     - Shadow moves into position. No feedbackself.
#     - Shadow grabs the bottle.
#         - Use Biotac feedback to properly grab the object?
#         - Use BT feedback to ensure the object is grabbed.
#             -   NN classification of "neutral" or "grabbed".
#     - Shadow moves to the release position.
#         - The BT proof that the object is still in its hand triggers Baxter
#            to start.
#     - Baxter will reach for the object (Different Script)
#         - Baxter uses its gripper feedback to ensure the object is grasped.
#     - Baxter pulls up on the object, causing a shadow classification that
#        slip is occuring.
#         - If the slip is upward, shadow will release the object.
#             - Using a moving average, normalized with reference. Based on
#                grasp_4_norm.
#             - Angular fitted classification.
#     - After Baxter has the object shadow returns to the start position.
#     - Baxter places the object in a new location outside of shadow's reach.
#


from __future__ import division
import time

import rospy
import numpy as np  # for: exp(), .shape, array, matrix
import numpy.matlib as npm  # for: npm.repmat()
import zmq
import msgpack

from std_msgs.msg import String, Float32, UInt8
from sr_robot_msgs.msg import BiotacAll

from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander
import slip_nn as snn


rospy.init_node("move_test", anonymous=True)

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5556")

hand_commander = SrHandCommander()
arm_commander = SrArmCommander()

ff = snn.BiotacData()
ff_elect_db = ff._elect_db
# ff_angle_mvavg = ff.angle_mvavg
# ff_pac1_mvavg = ff.pac1_mvavg
# ff_pdc_mvavg = ff.pdc_mvavg

# th = snn.BiotacData()

time.sleep(1)


# =============== Define Listen (Subscriber) Function ===============
def listen():
    rospy.Subscriber("/rh/tactile/", BiotacAll, callback)
    rospy.spin()


# =============== Define Callback Function ===============
def callback(data):
    ff_electrodes = list(data.tactiles[0].electrodes)  # comes in as a Tuple
    ff_pac1 = int(data.tactiles[0].pac1)  # append the Pac1 value
    ff_pdc = int(data.tactiles[0].pdc)
    if len(ff_elect_db) < 50:
        print("Establishing Pac1 and Electrode baselines...")
        ff.database(ff_electrodes, name="electrodes")
        ff.database(ff_pac1, name="pac1")
    else:
        features = np.array([0]*24)
        features[:24] = np.array(ff_electrodes[:24])
        features = np.matrix(features)  # convert list to numpy matrix
        # % diff from baseline:
        features = ((features.T - ff.elect_base)/ff.elect_base)*100
        angle = snn.fit_e24(features)  # must be a column matrix
        ff_pac1_zero = pac1_zero(ff_pac1)
        ff_pdc_zero = pdc_zero(ff_pdc)
        ff.new(angle, name="angle")
        ff.new(ff_pac1_zero, name="pac1")
        ff.new(ff_pdc_zero, name="pdc")
        check_grasp()  # check that SR has an object and send boolean to Baxter
        print("ff_angle: %+6.2f \t\t ff_pac1: %6.2f \t\t ff_pdc: %6.2f"
              % (ff.angle_mvavg, ff.pac1_mvavg, ff.pdc_mvavg))
        if (ff.pdc_mvavg > 50) and (ff.pac1_mvavg > 50) \
           and (170 < ff.angle_mvavg < 190):

            print("Upward slip detected!")
            joint_goals = hand_start  # Release the object
            hand_commander.move_to_joint_value_target_unsafe(
                joint_goals, 2, True)
            time.sleep(5)  # Provide enough time for Baxter to clear the area
            joint_goals = arm_start  # Move back to start
            arm_commander.move_to_joint_value_target_unsafe(
                joint_goals, 5, True)
            time.sleep(1)
            rospy.signal_shutdown("Slip was Detected")


# =============== Define Support Functions ===============
def pac1_zero(pac):
    b = pac-ff.pac1_base
    pac_zerod = np.abs(b)
    return(pac_zerod)


def pdc_zero(pdc):
    b = pdc-ff.pdc_base
    pdc_zerod = np.abs(b)
    return(pdc_zerod)


def pdc_callback(data):
    p = int(data.tactiles[0].pdc)
    ff.database(p, name="pdc")


def check_grasp():
    if (ff.pdc_mvavg > 50):  # SR has an object in its grasp
        msg = 1  # message to relay to Baxter informing of o
    else:
        msg = 0
    packed = msgpack.dumps(msg)
    socket.send(packed)


# =============== Define Position Dictionaries ===============
# --------------- Hand Positions ---------------
hand_start = {
        'rh_FFJ1': -0.013387468694274651,   'rh_FFJ2': 0.10550124582950798,
        'rh_FFJ3': -0.07913645703418956,    'rh_FFJ4': -0.020790969983510318,
        'rh_THJ4': 0.8987090669167258,      'rh_THJ5': -1.0529838245665772,
        'rh_THJ1': 0.36613957472880915,     'rh_THJ2': -0.3099264451304632,
        'rh_THJ3': 0.04339213288734181,     'rh_LFJ2': 0.31856120196799154,
        'rh_LFJ3': -0.13247924347682977,    'rh_LFJ1': 0.020856552138779016,
        'rh_LFJ4': 0.006156109478006114,    'rh_LFJ5': 0.030368858695598477,
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
# wrist joints removed from hand:
# start 'rh_WRJ2': -0.08740126759572807,'rh_WRJ1': -0.009642963029241673
# close :: 'rh_WRJ2': -0.103164843927744,      'rh_WRJ1': -0.10998772922135532

# --------------- Arm Positions ---------------
arm_start = {
    'ra_shoulder_pan_joint': -1.6755197683917444,
    'ra_elbow_joint': 2.391160726547241,
    'ra_wrist_1_joint': 2.303798198699951,
    'ra_shoulder_lift_joint': -1.5533440748797815,
    'ra_wrist_3_joint': -3.10664946237673,
    'rh_WRJ2': -0.08740126759572807,
    'rh_WRJ1': -0.009642963029241673,
    'ra_wrist_2_joint': -1.5882452170001429,
            }

arm_pickup = {
    'ra_shoulder_pan_joint': -0.575897518788473,
    'ra_elbow_joint': 2.86228346824646,
    'ra_wrist_1_joint': 1.6754974126815796,
    'ra_shoulder_lift_joint': -1.2914817968951624,
    'ra_wrist_3_joint': -1.5357773939715784,
    'rh_WRJ2': 0.05646164732737393,
    'rh_WRJ1': -0.10736475895393359,
    'ra_wrist_2_joint': -1.5881970564471644,
            }

arm_exit_pickup = {
    'ra_shoulder_pan_joint': -0.575909439717428,
    'ra_elbow_joint': 2.7576346397399902,
    'ra_wrist_1_joint': 1.8324915170669556,
    'ra_shoulder_lift_joint': -1.4485862890826624,
    'ra_wrist_3_joint': -1.5358369986163538,
    'rh_WRJ2': -0.008102103551746979,
    'rh_WRJ1': -0.10673035727744258,
    'ra_wrist_2_joint': -1.5882094542132776,
                }

arm_midway = {
    'ra_shoulder_pan_joint': -1.780236546193258,
    'ra_elbow_joint': 2.7576465606689453,
    'ra_wrist_1_joint': 1.8324674367904663,
    'ra_shoulder_lift_joint': -1.4485982100116175,
    'ra_wrist_3_joint': -1.5358369986163538,
    'rh_WRJ2': -0.008395394812687014,
    'rh_WRJ1': -0.10545759885212826,
    'ra_wrist_2_joint': -1.5882094542132776,
            }

arm_release = {
    'ra_shoulder_pan_joint': -3.027827803288595,
    'ra_elbow_joint': 2.6113691329956055,
    'ra_wrist_1_joint': 1.8882097005844116,
    'ra_shoulder_lift_joint': -1.3068426291095179,
    'ra_wrist_3_joint': -1.4986370245562952,
    'rh_WRJ2': -0.103164843927744,
    'rh_WRJ1': -0.10998772922135532,
    'ra_wrist_2_joint': -1.595231835042135,
            }

# =============== Main ===============
if __name__ == '__main__':
    # Move arm and hand to start position
    joint_goals = arm_start
    arm_commander.move_to_joint_value_target_unsafe(joint_goals, 5, True)
    joint_goals = hand_start
    hand_commander.move_to_joint_value_target_unsafe(joint_goals, 5, True)
    # ?? Check for object here using Biotacs ??
    while True:  # find pdc baseline
        print("Establishing Pdc baseline...")
        rospy.Subscriber("rh/tactile", BiotacAll, pdc_callback)
        if len(ff._pdc_db) >= 50:
            break

    # Move arm to pickup location
    joint_goals = arm_pickup
    arm_commander.move_to_joint_value_target_unsafe(joint_goals, 5, True)

    # Grab the object with the hand
    joint_goals = hand_close
    hand_commander.move_to_joint_value_target_unsafe(joint_goals, 4, True)

    # Exit the pickup zone
    joint_goals = arm_exit_pickup
    arm_commander.move_to_joint_value_target_unsafe(joint_goals, 3, True)

    # Go to the midway point
    joint_goals = arm_midway
    arm_commander.move_to_joint_value_target_unsafe(joint_goals, 5, False)

    # Go to the release area
    joint_goals = arm_release
    arm_commander.move_to_joint_value_target_unsafe(joint_goals, 5, True)

    time.sleep(0.5)
    listen()
