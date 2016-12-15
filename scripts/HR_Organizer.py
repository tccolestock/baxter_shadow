#!/usr/bin/env python


import rospy
from std_msgs.msg import String, Int8
import time
import numpy as np

from HR_Baxter import *

#from Objects import *
from HR_cases import *
# Global variables:

A = list() # empty list
B = list() # empty list
#obj1 = Object_1()  # assign Object_1 class to 
baxter = Baxter() # The object that controls Baxter. Defined in baxter.py.
Execute_cases = cases()

def initial_setup_baxter():
	"""
	Enable and set up baxter.

	"""
	
	print 'Initializing node...'
	rospy.init_node('organizer')
	baxter.enable()
	baxter.calibrateLeftGripper()
	baxter.calibraterightGripper()

	
def move_Arms_cases():

	pub1 = rospy.Publisher('/test_number', Int8, queue_size=10)
	pub2 = rospy.Publisher('/trust_level', Int8, queue_size=10)
	
	
	y = input("Please Enter your Number: ")
	A.append(y)
	
	# Publishing Test_subject_number
	test_number = A
        pub1.publish(test_number)
        
	Execute_cases.case_1()
	Execute_cases.case_2()
	Execute_cases.case_3()
	Execute_cases.case_4()
	Execute_cases.case_5()
	
	z = input("Please Enter the Trust level 1 to 5: where 5 is the highest : ")
	B.append(z)
	
	# Publishing Trust level
	trust_level = B
        pub2.publish(trust_level)
        
        
	
'''
###############################
def Pub_joint_position_speed():

	pub = rospy.Publisher('/joint_position_speed', Float32, queue_size=10)
        rate = rospy.Rate(10) # 10hz
        joint_speed = baxter."hello world %s" % rospy.get_time()
        pub.publish(joint_speed)
       
##################################
'''	
def grabobject():
	
	baxter.closeLeftGripper()
	
def openrightGripper():

	baxter.openrightGripper()
	
def releaseobject():
	
	baxter.openLeftGripper()

def organize():

	print 'Baxter Organizer initiated.'
	
	initial_setup_baxter()
	
	move_Arms_cases()
	
	
	
	
def disable_baxter():
	"""
	Disable Baxter.

	"""
	baxter.disable()



	
def main():
		
	while True:
		
		organize()
		
		
if __name__ == '__main__':
	main()
	rospy.spin()
