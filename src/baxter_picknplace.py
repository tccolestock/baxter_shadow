#!/usr/bin/env python



import rospkg
# rospy - ROS Python API
import rospy

from std_msgs.msg import String, Float32, UInt8, Int32

# baxter_interface - Baxter Python API
import baxter_interface

import time

import sys

from baxter_core_msgs.msg import EndEffectorState


# initialize our ROS node, registering it with the Master
rospy.init_node('Hello_Baxter', anonymous=True)

# create an instance of baxter_interface's Limb class
limb = baxter_interface.Limb('left')


#left_gripper = baxter_interface.Gripper('left') 
left_gripper = baxter_interface.Gripper('left') 

pos_left_1 = {'left_w0': -0.05713695628418789, 'left_w1': 0.9226993569579076, 'left_w2': -0.049285541247372826, 'left_e0': 0.08396207089761804, 'left_e1': 1.0664967686962072, 'left_s0': -0.6931669084004773, 'left_s1': -0.5552852059504804}

#pos_left_2 = {'left_w0': 1.467995584392454, 'left_w1': 1.173107693016037, 'left_w2': -2.3887381736894793, 'left_e0': 0.464700924881864, 'left_e1': 1.6018026945551622, 'left_s0': -0.8518044958053259, 'left_s1': -0.7203657675469214}

pos_left_2 = {'left_w0': 0.06866772415368388, 'left_w1': -1.2145364421895737, 'left_w2': -0.09639417383778828, 'left_e0': 0.25107731511903064, 'left_e1': 2.32742271056346, 'left_s0': -0.5697108134013171, 'left_s1': -1.093310363092024}


pos_left_3 = {'left_w0': -0.007610971625083771, 'left_w1': -0.6845449098915354, 'left_w2': -0.25758326857710373, 'left_e0': 0.27017757905909384, 'left_e1': 1.0300837227358586, 'left_s0': -0.5329632719467685, 'left_s1': -0.4199615268238289}


#pos_left_3 = {'left_w0': -0.04536282798285573, 'left_w1': -0.5004391161654674, 'left_w2': -0.21157225292953738, 'left_e0': 0.27508543298012267, 'left_e1': 1.0127857393418092, 'left_s0': -0.5620855417838021, 'left_s1': -0.4975892551042004}

#pos_left_4 = {'left_w0': -0.03204075728758501, 'left_w1': 0.034888899794307836, 'left_w2': -0.27208016378934774, 'left_e0': 0.40153459644515826, 'left_e1': 0.8966282367852958, 'left_s0': -0.6588519146251455, 'left_s1': -0.5990253236285605}

pos_left_4 = {'left_w0': -0.04466524691419955, 'left_w1': -0.6769642311008549, 'left_w2': -0.20603977248860436, 'left_e0': 0.287585842044487, 'left_e1': 1.0601728933314833, 'left_s0': -0.5649041029080709, 'left_s1': -0.5238381998133573}

pos_left_5 = {'left_w0': 0.030677445526000883, 'left_w1': -0.41686560250084576, 'left_w2': -0.31214287138384367, 'left_e0': 0.3750361463957424, 'left_e1': 1.156250404109273, 'left_s0': -0.6473285021007186, 'left_s1': -0.6906852092110475}


pos_left_6 = {'left_w0': 0.018939177536936854, 'left_w1': -0.09549088120269449, 'left_w2': -0.2083347912294664, 'left_e0': 0.3343205062481607, 'left_e1': 1.1662143651695107, 'left_s0': -0.6472867724086893, 'left_s1': -0.9338264750851822}

pos_left_7 = {'left_w0': 0.3137361774130394, 'left_w1': -0.7862804325774785, 'left_w2': -0.257034977412297, 'left_e0': -0.11775130706005019, 'left_e1': 1.6701363578086654, 'left_s0': -1.0049116853267022, 'left_s1': -0.9089321667435579}



#pos_left_7 = {'left_w0': 0.0615142316063285, 'left_w1': -0.5664148156090704, 'left_w2': -0.2133921320437357, 'left_e0': 0.5741484346702467, 'left_e1': 1.7181217938092623, 'left_s0': -0.38645138909770366, 'left_s1': -1.0648001879708806}


pos_left_8 = {'left_w0': 0.30331995568180103, 'left_w1': -1.1268090067854226, 'left_w2': -0.03908113899786168, 'left_e0': -0.10953135192020516, 'left_e1': 1.1481916322704702, 'left_s0': -1.1512396184980704, 'left_s1': -0.005415235176801978}

#pos_left_8 = {'left_w0': 0.17650681664250367, 'left_w1': -0.9767386794791614, 'left_w2': -0.34688814512779037, 'left_e0': 0.2616928793930192, 'left_e1': 1.353367922026508, 'left_s0': -0.043175223640717665, 'left_s1': -0.3030130932855728}


#pos_left_9 = {'left_w0': -0.4597372748691515, 'left_w1': -1.3199924973368022, 'left_w2': -1.1168413163051565, 'left_e0': 0.9608910579805319, 'left_e1': 1.9942061282873869, 'left_s0': -0.810451331061034, 'left_s1': 0.15273044535739977}

# set_joint_position_speed(self, speed)


def callback1(data):

	global grasp_object

	x = data.gripping
	 
	grasp_object = x
	
	print (grasp_object)

def callback(data):
	d = data.data
	print(d)
	
	if d == 1:

		#left_gripper.open()
		left_gripper.calibrate()

		limb.set_joint_position_speed(0.3)

		#limb.move_to_joint_positions(pos_test)
		left_gripper.open()

		time.sleep(2)

		limb.move_to_joint_positions(pos_left_1)
		
		time.sleep(1)

		limb.move_to_joint_positions(pos_left_2)
		
		left_gripper.calibrate()

		time.sleep(1)

		limb.set_joint_position_speed(0.2)

		limb.move_to_joint_positions(pos_left_3)

		time.sleep (1)

		left_gripper.close()
		
		time.sleep (1)
		
		if (grasp_object == 1):
		

			time.sleep (1)

			limb.set_joint_position_speed(0.05)

			limb.move_to_joint_positions(pos_left_4)

			time.sleep (1)

			limb.move_to_joint_positions(pos_left_5)

			limb.move_to_joint_positions(pos_left_6)

			limb.set_joint_position_speed(0.3)

			limb.move_to_joint_positions(pos_left_7)

			limb.move_to_joint_positions(pos_left_8)
		
			time.sleep (1)

			left_gripper.open()

			time.sleep (1)
		
			limb.move_to_joint_positions(pos_left_7)

			limb.move_to_joint_positions(pos_left_1)
			
			rospy.signal_shutdown("done")
			
			
		else:
			
			print (" gripper not detecting any object ")
			
			left_gripper.open()
			
			time.sleep(1)

			limb.set_joint_position_speed(0.2)
			
			limb.move_to_joint_positions(pos_left_2)
			
			left_gripper.calibrate()

			time.sleep(1)

			limb.move_to_joint_positions(pos_left_3)

			time.sleep (1)

			left_gripper.close()
			
			time.sleep (1)
			
			if (grasp_object == 1):
		

				limb.set_joint_position_speed(0.05)

				limb.move_to_joint_positions(pos_left_4)

				time.sleep (1)

				limb.move_to_joint_positions(pos_left_5)

				limb.move_to_joint_positions(pos_left_6)


				limb.set_joint_position_speed(0.3)

				limb.move_to_joint_positions(pos_left_7)

				limb.move_to_joint_positions(pos_left_8)
		
				time.sleep (1)

				left_gripper.open()

				time.sleep (1)
		
				limb.move_to_joint_positions(pos_left_7)

				limb.move_to_joint_positions(pos_left_1)
				
			else:
			
				print (" gripper not detecting any object ")
			
				time.sleep (1)
			
				left_gripper.open()
			
				time.sleep(1)

				limb.move_to_joint_positions(pos_left_2)
			
				left_gripper.calibrate()

				time.sleep(1)

				limb.move_to_joint_positions(pos_left_3)

				time.sleep (1)

				left_gripper.close()
				
				time.sleep (1)
				
				if (grasp_object == 1):
		

					limb.set_joint_position_speed(0.05)

					limb.move_to_joint_positions(pos_left_4)

					time.sleep (1)

					limb.move_to_joint_positions(pos_left_5)

					limb.move_to_joint_positions(pos_left_6)


					limb.set_joint_position_speed(0.3)

					limb.move_to_joint_positions(pos_left_7)

					limb.move_to_joint_positions(pos_left_8)
		
					time.sleep (1)

					left_gripper.open()

					time.sleep (1)
		
					limb.move_to_joint_positions(pos_left_7)

					limb.move_to_joint_positions(pos_left_1)
					
				else: 
			
					print (" gripper not detecting any object ")
					
					left_gripper.open()
			
					time.sleep (1)
			
						
					limb.move_to_joint_positions(pos_left_2)
					
					time.sleep (1)
					
					limb.move_to_joint_positions(pos_left_1)
					
					rospy.signal_shutdown("nothing")
					
			#rospy.signal_shutdown ('end')

	else:
		print (" The object is not in position")

def listen():

    rospy.Subscriber("/sr_grip_logic",UInt8 ,callback, queue_size=1)
    rospy.Subscriber("/robot/end_effector/left_gripper/state",EndEffectorState ,callback1)
    #pub = rospy.Publisher("topic_name", Message_Type, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    listen()

