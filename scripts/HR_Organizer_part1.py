#!/usr/bin/env python


import rospy
from std_msgs.msg import String, Int8
import time
import numpy as np

from HR_Baxter import *

#from Objects import *
from HR_cases_1 import *
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
	baxter.set_holding_force ()
	
def move_Arms_cases():

	pub1 = rospy.Publisher('/subject_number', Int8, queue_size=10, latch=True)
	pub2 = rospy.Publisher('/trust_level', Int8, queue_size=10, latch=True)
	
	
	subject_number = int(raw_input("Please Enter your Number: "))
	
	trust_level = int(raw_input("Please Enter the Trust level 1 to 5: where 5 is the highest : "))
	
	
	pub1.publish(subject_number)
	pub2.publish(trust_level)

	
        
        
	Execute_cases.case_1()
	trust_level = int(raw_input("Please Enter the Trust level 1 to 5: where 5 is the highest : "))
	pub2.publish(trust_level)
	
	
	Execute_cases.case_2()
	trust_level = int(raw_input("Please Enter the Trust level 1 to 5: where 5 is the highest : "))
	pub2.publish(trust_level)
	
	
	Execute_cases.case_3()
	trust_level = int(raw_input("Please Enter the Trust level 1 to 5: where 5 is the highest : "))
	pub2.publish(trust_level)
	
	Execute_cases.case_4()
	trust_level = int(raw_input("Please Enter the Trust level 1 to 5: where 5 is the highest : "))
	pub2.publish(trust_level)
	
	
	Execute_cases.case_5()
	trust_level = int(raw_input("Please Enter the Trust level 1 to 5: where 5 is the highest : "))
	pub2.publish(trust_level)
	
	
	Execute_cases.case_6()
	trust_level = int(raw_input("Please Enter the Trust level 1 to 5: where 5 is the highest : "))
	pub2.publish(trust_level)
	
	
	Execute_cases.case_7()
	trust_level = int(raw_input("Please Enter the Trust level 1 to 5: where 5 is the highest : "))
	pub2.publish(trust_level)
	
	
	Execute_cases.case_8()
	trust_level = int(raw_input("Please Enter the Trust level 1 to 5: where 5 is the highest : "))
	pub2.publish(trust_level)
	
	
	Execute_cases.case_9()
	trust_level = int(raw_input("Please Enter the Trust level 1 to 5: where 5 is the highest : "))
	pub2.publish(trust_level)
      
	

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
	#rospy.spinOnce()
	#rospy.spin()
