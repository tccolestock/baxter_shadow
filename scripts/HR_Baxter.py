#!/usr/bin/env python
import argparse
import struct
import sys
import rospy
from std_msgs.msg import Header,UInt16, Float32
import numpy as np
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface
from geometry_msgs.msg import (
	PoseStamped,
	Pose,
	Point,
	Quaternion,
)

#import Objects

# create an instance of baxter_interface's Limb class
#rospy.init_node('Pick_and_Place', anonymous=True)
#limb_right = baxter_interface.Limb('right')
#limb_left = baxter_interface.Limb('left')

p = Pose()



class Baxter:
	def __init__(self):
		# Baxter arms speed cases
		self.lowspeed = 0.1
		self.rightspeed = 0.3
		self.highspeed = 0.7
		#pass

	def enable(self):
		# An attribute of class RobotEnable to enable or disable Baxter.
		# It has the following methods:
		#	enable()	- enable all joints
		#	disable()	- disable all joints
		#	reset()	- reset all joints, reset all jrcp faults, disable the robot
		#	stop()		- stop the robot, similar to hitting the e-stop button
		self.robotEnable = baxter_interface.RobotEnable()
		self.robotEnable.enable()
		self.leftArm = baxter_interface.Limb('left')
		self.rightArm = baxter_interface.Limb('right')
		
		# Waits for the image service of right hand camera to become available.
		self.leftGripper = baxter_interface.Gripper('left')
		self.rightGripper = baxter_interface.Gripper('right')
		
	def disable(self):
		self.robotEnable.disable()
		
	def getLeftArmPosition(self):
		position = self.leftArm.endpoint_pose()
		x = position['position'].x
		y = position['position'].y
		z = position['position'].z
		return [x,y,z]

	def getLeftArmOrientation(self):
		orientation = self.leftArm.endpoint_pose()
		x = orientation['orientation'].x
		y = orientation['orientation'].y
		z = orientation['orientation'].z
		w = orientation['orientation'].w
		return [x,y,z,w]
		
	def getRightArmPosition(self):
		#position = self.rightArm.endpoint_pose()
		#x = position['position'].x
		#y = position['position'].y
		#z = position['position'].z
		p.position.x = Objects.Object_1.object1_position[0]
		p.position.y = Objects.Object_1.object1_position[1]
		p.position.z = Objects.Object_1.object1_position[2]
		
		x = p.position.x
		y = p.position.y
		z = p.position.z
				
		return x, y, z
		
	
	def getRightArmOrientation(self):
		#orientation = self.rightArm.endpoint_pose()
		#x = orientation['orientation'].x
		#y = orientation['orientation'].y
		#z = orientation['orientation'].z
		#w = orientation['orientation'].w
		p.orientation.x = Objects.Object_1.object1_orintation[0]
		p.orientation.y = Objects.Object_1.object1_orintation[1]
		p.orientation.z = Objects.Object_1.object1_orintation[2]
		p.orientation.w = Objects.Object_1.object1_orintation[3]
		x = p.orientation.x
		y = p.orientation.y		
		z = p.orientation.z
		w = p.orientation.w
		
		return x, y, z, w
		
	def inverseKinematics(self, limb, point, orientation):
		
    		ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    		iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    		ikreq = SolvePositionIKRequest()
		hdr = Header(stamp=rospy.Time.now(), frame_id='base')
		pose = PoseStamped(
					header=hdr,
					pose=Pose(
					position=Point(
						x=point[0],
						y=point[1],
						z=point[2],
					),
					orientation=Quaternion(
						x=orientation[0],
						y=orientation[1],
						z=orientation[2],
						w=orientation[3],
					)
				)
			);
		ikreq.pose_stamp.append(pose)
		try:
        		rospy.wait_for_service(ns, 5.0)
        		resp = iksvc(ikreq)
    		except (rospy.ServiceException, rospy.ROSException), e:
        		rospy.logerr("Service call failed: %s" % (e,))
       			return 1

    		# Check if result valid, and type of seed ultimately used to get solution
    		# convert rospy's string representation of uint8[]'s to int's
    		resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                             		  resp.result_type)
    		if (resp_seeds[0] != resp.RESULT_INVALID):
        		seed_str = {
                    			ikreq.SEED_USER: 'User Provided Seed',
                    			ikreq.SEED_CURRENT: 'Current Joint Angles',
                    			ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   			}.get(resp_seeds[0], 'None')
        		print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
             			 (seed_str,))
        # Format solution into Limb API-compatible dictionary
        		limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
       			print "\nIK Joint Solution:\n", limb_joints
        		print "------------------"
        		print "Response Message:\n", resp
   
			return limb_joints
		else:
			print("INVALID POSE - No Valid Joint Solution Found.")
			return None
			
			
	def ik(self, point, orientation):
		angles = self.inverseKinematics('left', point, orientation)
		return angles
			
		
	def moveLeftArm(self, point, orientation):
		print 'inmoveleftArm Method'
		print 'point =', point
		print 'orientation =', orientation
		angles = self.inverseKinematics('left', point, orientation)
		print 'angles=' , angles
		if not angles:
			return None
		self.leftArm.move_to_joint_positions(angles) # 15 secs timeout default.
	def moverightArm(self, point, orientation):
		angles = self.inverseKinematics('right', point, orientation)
		if not angles:
			return None
		self.rightArm.move_to_joint_positions(angles) # 15 secs timeout default.
		
		
		
#######################################################################################		
	
	def setjointspeed(self, speed):
		self.x = speed
		limb_right = baxter_interface.Limb('right')
		limb_left = baxter_interface.Limb('left')
		
		limb_left.set_joint_position_speed(self.x)
		limb_right.set_joint_position_speed(self.x)
		
	def set_holding_force (self):
		right_gripper = baxter_interface.Gripper('right')
		left_gripper = baxter_interface.Gripper('left')
		
		right_gripper.set_holding_force(50)	
		left_gripper.set_holding_force(50)
		
				
		
	def rightGripperpos(self):
		right_gripper = baxter_interface.Gripper('right')
		H = right_gripper.position()
		
		return H
		
	def move_right_arm(self, angles):
		limb_right = baxter_interface.Limb('right')
		limb_right.move_to_joint_positions(angles)
	
	
	def move_left_arm(self, angles):
		limb_left = baxter_interface.Limb('left')
		limb_left.move_to_joint_positions(angles)	
		
	# Gripper Methods:
	def calibraterightGripper(self):
		self.rightGripper.calibrate()

		
	def closerightGripper(self):
		right_gripper = baxter_interface.Gripper('right')
		right_gripper.close()
		
	def openrightGripper(self):
		right_gripper = baxter_interface.Gripper('right')
		right_gripper.open()
		
	def calibrateLeftGripper(self):
		self.leftGripper.calibrate()
		
	def closeleftGripper(self):
		left_gripper = baxter_interface.Gripper('left')
		left_gripper.close()
		
	def openleftGripper(self):
		
		left_gripper = baxter_interface.Gripper('left')
		left_gripper.open()
