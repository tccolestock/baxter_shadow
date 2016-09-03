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

from std_msgs.msg import String, Float32, UInt8, Int16
from sr_robot_msgs.msg import BiotacAll, Biotac
from baxter_shadow.srv import NeuralNetwork, PdcBaseline

from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander
import slip_nn as snn


rospy.init_node("move_test", anonymous=True)

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5556")

hand_commander = SrHandCommander()
arm_commander = SrArmCommander()

rate_handle = rospy.Rate(100) #hz

# Define publishers
pub_angle = rospy.Publisher("ff_angle", Float32, queue_size=10)
pub_pac1 = rospy.Publisher("ff_pac1", Float32, queue_size=10)
pub_pdc = rospy.Publisher("ff_pdc", Float32, queue_size=10)

# Define Services
neural_network = rospy.ServiceProxy("neural_network", NeuralNetwork)

np_array = np.array
np_matrix = np.matrix
np_abs = np.abs
np_multiply = np.multiply
np_exp = np.exp
np_dot = np.dot
npm_repmat = npm.repmat

ff = snn.BiotacData()
ff_elect_db = ff._elect_db
ff_elect_base = ff.elect_base
ff_pac1_base = ff.pac1_base
ff_pdc_base = ff.pdc_base
ff_new = ff.new
ff_angle_mvavg = ff.angle_mvavg
ff_pac1_mvavg = ff.pac1_mvavg
ff_pdc_mvavg = ff.pdc_mvavg

# th = snn.BiotacData()

time.sleep(0.1)


# =============== Define Listen (Subscriber) Function ===============
def listen():
    rospy.Subscriber("/rh/tactile/", BiotacAll, callback, queue_size=10)
    rospy.spin()


# =============== Define Callback Function ===============
def callback(data):
    ff_electrodes = list(data.tactiles[0].electrodes)  # comes in as a Tuple
    ff_pac1 = int(data.tactiles[0].pac1)  # append the Pac1 value
    ff_pdc = int(data.tactiles[0].pdc)
    if len(ff._elect_db) < 50:
        print("Establishing Pac1 and Electrode baselines...")
        ff.database(ff_electrodes, name="electrodes")
        ff.database(ff_pac1, name="pac1")
    else:
        features = [((e - b)/b)*100 for e, b in zip(ff_electrodes, ff_elect_base)]
        response = neural_network(features)
        angle = response.output
        ff_pac1_zero = zero_value(ff_pac1, ff_pac1_base[0])
        ff_pdc_zero = zero_value(ff_pdc, ff_pdc_base[0])
        ff_new(angle, name="angle")
        ff_new(ff_pac1_zero, name="pac1")
        ff_new(ff_pdc_zero, name="pdc")
        check_grasp()  # check that SR has an object and send boolean to Baxter
        print("ff_angle: %+6.2f \t\t ff_pac1: %6.2f \t\t ff_pdc: %6.2f"
              % (ff_angle_mvavg[0], ff_pac1_mvavg[0], ff_pdc_mvavg[0]))

        pub_angle.publish(ff_angle_mvavg[0])
        pub_pac1.publish(ff_pac1_mvavg[0])
        pub_pdc.publish(ff_pdc_mvavg[0])

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
def zero_value(x, base):
    return np_abs(x-base)


def baseline_pdc():
    rospy.wait_for_service('pdc_baseline')
    pdc_baseline_handle = rospy.ServiceProxy("pdc_baseline", PdcBaseline)
    pdc_resp = pdc_baseline_handle()
    ff_pdc_base[0] = pdc_resp.baseline


def check_grasp():
    if (ff_pdc_mvavg[0] > 50):  # SR has an object in its grasp
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
    'ra_shoulder_pan_joint': -0.64500271116839808,
    'ra_shoulder_lift_joint': -1.2458885847674769,
    'ra_elbow_joint': 2.7204307918548584,
    'ra_wrist_1_joint': 1.6558868989944458,
    'ra_wrist_2_joint': -1.640662972127096,
    'ra_wrist_3_joint': -1.5906219450580042,
    'rh_WRJ1': -0.0851542173817335,
    'rh_WRJ2': -0.02776222781084232
    }

arm_exit_pickup = {
    'ra_shoulder_pan_joint': -0.64500271116839808,
    'ra_shoulder_lift_joint': -1.6580899397479456,
    'ra_elbow_joint': 2.652906656265259,
    'ra_wrist_1_joint': 2.129228115081787,
    'ra_wrist_2_joint': -1.640674893056051,
    'ra_wrist_3_joint': -1.5906219450580042,
    'rh_WRJ1': -0.07590067860170616,
    'rh_WRJ2': -0.02776222781084232
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
    # # Move arm and hand to start position
    # joint_goals = arm_start
    # arm_commander.move_to_joint_value_target_unsafe(joint_goals, 5, True)
    # joint_goals = hand_start
    # hand_commander.move_to_joint_value_target_unsafe(joint_goals, 5, True)

    # ?? Check for object here using Biotacs ??
    baseline_pdc()

    # # Move arm to pickup location
    # joint_goals = arm_pickup
    # arm_commander.move_to_joint_value_target_unsafe(joint_goals, 5, True)
    #
    # # Grab the object with the hand
    # joint_goals = hand_close
    # hand_commander.move_to_joint_value_target_unsafe(joint_goals, 4, True)
    #
    # # Exit the pickup zone
    # joint_goals = arm_exit_pickup
    # arm_commander.move_to_joint_value_target_unsafe(joint_goals, 3, True)
    #
    # # Go to the midway point
    # joint_goals = arm_midway
    # arm_commander.move_to_joint_value_target_unsafe(joint_goals, 5, False)
    #
    # # Go to the release area
    # joint_goals = arm_release
    # arm_commander.move_to_joint_value_target_unsafe(joint_goals, 5, True)

    # time.sleep(0.5)
    rospy.wait_for_service('neural_network')
    listen()
