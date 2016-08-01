#!/usr/bin/env python

from __future__ import division
import rospy
from std_msgs.msg import String, Float32, UInt8
# from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_robot_msgs.msg import BiotacAll
import time

rospy.init_node("sr_grasp", anonymous=True)

# arm_commander = SrArmCommander()
hand_commander = SrHandCommander()
time.sleep(1)

# hand positions:::
start = {'rh_FFJ1': -0.013387468694274651, 'rh_FFJ2': 0.10550124582950798, 'rh_FFJ3': -0.07913645703418956, 'rh_FFJ4': -0.020790969983510318, 'rh_THJ4': 0.8987090669167258, 'rh_THJ5': -1.0529838245665772, 'rh_THJ1': 0.36613957472880915, 'rh_THJ2': -0.3099264451304632, 'rh_THJ3': 0.04339213288734181, 'rh_LFJ2': 0.31856120196799154, 'rh_LFJ3': -0.13247924347682977, 'rh_LFJ1': 0.020856552138779016, 'rh_LFJ4': 0.006156109478006114, 'rh_LFJ5': 0.030368858695598477, 'rh_RFJ4': -0.017502072148899307, 'rh_RFJ1': 0.04862574836081379, 'rh_RFJ2': 0.23106641618794493, 'rh_RFJ3': -0.040169677117662395, 'rh_MFJ1': 0.0061621824517631985, 'rh_MFJ3': -0.03814186780706377, 'rh_MFJ2': 0.28535536916148746, 'rh_MFJ4': 0.005735133335643892, 'rh_WRJ2': -0.08740126759572807, 'rh_WRJ1': -0.009642963029241673}

close = {'rh_FFJ1': 0.5366228138727492, 'rh_FFJ2': 1.3707472622836295, 'rh_FFJ3': 0.6104890181588297, 'rh_FFJ4': -0.1693188654196813, 'rh_THJ4': 1.1494816044032174, 'rh_THJ5': -0.25236240595266746, 'rh_THJ1': 1.0564478227578378, 'rh_THJ2': 0.5591902548242037, 'rh_THJ3': 0.3010860128238289, 'rh_LFJ2': 1.1510589476677358, 'rh_LFJ3': 0.3496450123403709, 'rh_LFJ1': 0.2812655031286765, 'rh_LFJ4': 0.0007317935784767475, 'rh_LFJ5': 0.038378063907728126, 'rh_RFJ4': -0.030822436892029084, 'rh_RFJ1': 0.2252787835450361, 'rh_RFJ2': 1.1696882711839942, 'rh_RFJ3': 0.6358242015720096, 'rh_MFJ1': 0.18990725919524606, 'rh_MFJ3': 0.6792600589796994, 'rh_MFJ2': 1.3251573950327318, 'rh_MFJ4': -0.007377111269187729, 'rh_WRJ2': -0.103164843927744, 'rh_WRJ1': -0.10998772922135532}

soft_open = {'rh_FFJ1': 0.015299964222028228, 'rh_FFJ2': 1.0363475765685581, 'rh_FFJ3': 0.2156981911673815, 'rh_FFJ4': -0.09041898402453244, 'rh_THJ4': 1.1566064166609298, 'rh_THJ5': -0.4976068025062665, 'rh_THJ1': 0.7331455374652653, 'rh_THJ2': 0.24076301002605377, 'rh_THJ3': 0.2482866853523483, 'rh_LFJ2': 0.9579282517503304, 'rh_LFJ3': 0.22891073506641474, 'rh_LFJ1': 0.0369458923601228, 'rh_LFJ4': -0.010122565656606665, 'rh_LFJ5': 0.03884889211514442, 'rh_RFJ4': -0.03515217103578468, 'rh_RFJ1': 0.06709122242188231, 'rh_RFJ2': 0.8408973912178247, 'rh_RFJ3': 0.34325412649756837, 'rh_MFJ1': 0.014565158522349297, 'rh_MFJ3': 0.4407150002695516, 'rh_MFJ2': 0.7245574605990543, 'rh_MFJ4': -0.005447683603367941, 'rh_WRJ2': -0.106417846398269, 'rh_WRJ1': -0.07804339747071865}


def callback(data):
    ffe1 = data.tactiles[0].electrodes[0]
    # ffe2 = data.tactiles[0].electrodes[1]
    ffe3 = data.tactiles[0].electrodes[2]
    ffe4 = data.tactiles[0].electrodes[3]
    if (ffe1 < 3550) and (ffe3 < 3600) and (ffe4 < 3650):
        # hand_commander.move_to_joint_value_target_unsafe(soft_open, 1, True)
        hand_commander.move_to_joint_value_target_unsafe(start, 1, True)
        time.sleep(1)
        rospy.signal_shutdown("Slip was Detected")



def listen():
    rospy.Subscriber("rh/tactile/", BiotacAll, callback)
    rospy.spin()

hand_commander.move_to_joint_value_target_unsafe(start, 3, True)
hand_commander.move_to_joint_value_target_unsafe(close, 3, True)
time.sleep(2)

# listen()






# def talker():
#
#     hand_commander.move_to_joint_value_target_unsafe
#
# if __name__ == '__main__':
#     talker()
