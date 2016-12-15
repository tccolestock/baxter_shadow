#!/usr/bin/env python


#import roslib; roslib.load_manifest('organizer')
import rospy
#import iodevices
import time
import numpy as np
from std_msgs.msg import String, Float32
from HR_Baxter import *

from HR_Objects import *
# Global variables:


obj1 = Object_1()  # assign Object_1 class to obj1
obj2 = Object_2()
obj3 = Object_3()
obj4 = Object_4()
obj5 = Object_5()
obj6 = Object_6()
obj7 = Object_7()
obj8 = Object_8()
obj9 = Object_9()
wrong = Object_Wronglocation()

baxter = Baxter() # The object that controls Baxter. Defined in baxter.py.

# initialize our ROS node, registering it with the Master
#rospy.init_node('Hello_Baxter', anonymous=True)

# create an instance of baxter_interface's Limb class
#right_limb = baxter_interface.Limb('right')
#left_limb = baxter_interface.Limb('left')

#right_gripper = baxter_interface.Gripper('right')
#right_gripper = baxter_interface.Gripper('right')
#left_gripper = baxter_interface.Gripper('left')



class cases (object):

	def __init__(self):
		pass
		
#####################################################################################
# Objects 1, 2 and 3 in the right speed right location
	def case_1(self): 
		self.x = baxter.rightspeed
	
		print "Robot_speed = %s" % self.x 
		
		# Publishing joint_position_speed
		joint_speed = self.x

        	# Set joint position speed
		baxter.setjointspeed(self.x)
		
###################### object 1 execution

		rospy.loginfo('Picking Object 1')
		

		pos = obj1.pos_right_1()
		baxter.move_right_arm(pos)
		
		pos = obj1.pos_left_1()
		baxter.move_left_arm(pos)
		
		pos = obj1.pos_left_2()
		baxter.move_left_arm(pos)
		
		time.sleep (0.5)
		baxter.openleftGripper()

		pos = obj1.pos_left_3()
		baxter.move_left_arm(pos)
		time.sleep (0.5)
		baxter.closeleftGripper()
		time.sleep (0.5)
		
		pos = obj1.pos_left_4()
		baxter.move_left_arm(pos)
		
		pos = obj1.pos_left_5()
		baxter.move_left_arm(pos)
		
		pos = obj1.pos_right_2()
		baxter.move_right_arm(pos)
		time.sleep (0.5)
		baxter.openrightGripper()
		
		pos = obj1.pos_right_3()
		baxter.move_right_arm(pos)
		
		baxter.closerightGripper()
		time.sleep (1)
		baxter.openleftGripper()
		time.sleep (0.5)
		
		pos = obj1.pos_left_6()
		baxter.move_left_arm(pos)
		
		pos = obj1.pos_right_4()
		baxter.move_right_arm(pos)		
				
		
		H = baxter.rightGripperpos()
		print "H = %s" % H 
		while  (H < 50):
			H = baxter.rightGripperpos()
			time.sleep(1)
		pos = obj1.pos_right_1()
		baxter.move_right_arm(pos)
		
##################### object 2 execution
		rospy.loginfo('Picking Object 2')
		

		pos = obj2.pos_right_1()
		baxter.move_right_arm(pos)
		
		pos = obj2.pos_left_1()
		baxter.move_left_arm(pos)
		
		pos = obj2.pos_left_2()
		baxter.move_left_arm(pos)
		
		time.sleep (0.5)
		baxter.openleftGripper()

		pos = obj2.pos_left_3()
		baxter.move_left_arm(pos)
		time.sleep (0.5)
		baxter.closeleftGripper()
		time.sleep (0.5)
		
		pos = obj2.pos_left_4()
		baxter.move_left_arm(pos)
		
		pos = obj2.pos_left_5()
		baxter.move_left_arm(pos)
		
		pos = obj2.pos_right_2()
		baxter.move_right_arm(pos)
		time.sleep (0.5)
		baxter.openrightGripper()
		
		pos = obj2.pos_right_3()
		baxter.move_right_arm(pos)
		
		baxter.closerightGripper()
		time.sleep (1)
		baxter.openleftGripper()
		time.sleep (0.5)
		
		pos = obj2.pos_left_6()
		baxter.move_left_arm(pos)
		
		pos = obj2.pos_right_4()
		baxter.move_right_arm(pos)		
					
		H = baxter.rightGripperpos()
		print "H = %s" % H 
		while  (H < 50):
			H = baxter.rightGripperpos()
			time.sleep(1)
		pos = obj2.pos_right_1()
		baxter.move_right_arm(pos)
		
##################### object 3 execution
		rospy.loginfo('Picking Object 3')
		

		pos = obj3.pos_right_1()
		baxter.move_right_arm(pos)
		
		pos = obj3.pos_left_1()
		baxter.move_left_arm(pos)
		
		pos = obj3.pos_left_2()
		baxter.move_left_arm(pos)
		
		time.sleep (0.5)
		baxter.openleftGripper()

		pos = obj3.pos_left_3()
		baxter.move_left_arm(pos)
		time.sleep (0.5)
		baxter.closeleftGripper()
		time.sleep (0.5)
		
		pos = obj3.pos_left_4()
		baxter.move_left_arm(pos)
		
		pos = obj3.pos_left_5()
		baxter.move_left_arm(pos)
		
		pos = obj3.pos_right_2()
		baxter.move_right_arm(pos)
		time.sleep (0.5)
		baxter.openrightGripper()
		
		pos = obj3.pos_right_3()
		baxter.move_right_arm(pos)
		
		baxter.closerightGripper()
		time.sleep (1)
		baxter.openleftGripper()
		time.sleep (0.5)
		
		pos = obj3.pos_left_6()
		baxter.move_left_arm(pos)
		
		pos = obj3.pos_right_4()
		baxter.move_right_arm(pos)		
					
		H = baxter.rightGripperpos()
		print "H = %s" % H 
		while  (H < 50):

			H = baxter.rightGripperpos()
			time.sleep(1)
		pos = obj3.pos_right_1()
		baxter.move_right_arm(pos)	

		
#######################################################################################		
# Objects 4, 5, and 6 low speed  and right location
	def case_2(self): 

		self.x = baxter.lowspeed
	
		print "Robot_speed = %s" % self.x
		 
		# Publishing joint_position_speed
		
		joint_speed = self.x

        	
		# Set joint position speed	
		baxter.setjointspeed(self.x)
		
		
##################### object 4 execution
		rospy.loginfo('Picking Object 4')
		

		pos = obj4.pos_right_1()
		baxter.move_right_arm(pos)
		
		pos = obj4.pos_left_1()
		baxter.move_left_arm(pos)
		
		pos = obj4.pos_left_2()
		baxter.move_left_arm(pos)
		
		time.sleep (0.5)
		baxter.openleftGripper()

		pos = obj4.pos_left_3()
		baxter.move_left_arm(pos)
		time.sleep (0.5)
		baxter.closeleftGripper()
		time.sleep (0.5)
		
		pos = obj4.pos_left_4()
		baxter.move_left_arm(pos)
		
		pos = obj4.pos_left_5()
		baxter.move_left_arm(pos)
		
		pos = obj4.pos_right_2()
		baxter.move_right_arm(pos)
		time.sleep (0.5)
		baxter.openrightGripper()
		
		pos = obj4.pos_right_3()
		baxter.move_right_arm(pos)
		
		baxter.closerightGripper()
		time.sleep (1)
		baxter.openleftGripper()
		time.sleep (0.5)
		
		pos = obj4.pos_left_6()
		baxter.move_left_arm(pos)
		
		pos = obj4.pos_right_4()
		baxter.move_right_arm(pos)		
					
		H = baxter.rightGripperpos()
		print "H = %s" % H 
		while  (H < 50):
			H = baxter.rightGripperpos()
			time.sleep(1)
		pos = obj4.pos_right_1()
		baxter.move_right_arm(pos)
		
		
##################### object 5 execution
	
		rospy.loginfo('Picking Object 5')
		

		pos = obj5.pos_right_1()
		baxter.move_right_arm(pos)
		
		pos = obj5.pos_left_1()
		baxter.move_left_arm(pos)
		
		pos = obj5.pos_left_2()
		baxter.move_left_arm(pos)
		
		time.sleep (0.5)
		baxter.openleftGripper()

		pos = obj5.pos_left_3()
		baxter.move_left_arm(pos)
		time.sleep (0.5)
		baxter.closeleftGripper()
		time.sleep (0.5)
		
		pos = obj5.pos_left_4()
		baxter.move_left_arm(pos)
		
		pos = obj5.pos_left_5()
		baxter.move_left_arm(pos)
		
		pos = obj5.pos_right_2()
		baxter.move_right_arm(pos)
		time.sleep (0.5)
		baxter.openrightGripper()
		
		pos = obj5.pos_right_3()
		baxter.move_right_arm(pos)
		
		baxter.closerightGripper()
		time.sleep (1)
		baxter.openleftGripper()
		time.sleep (0.5)
		
		pos = obj5.pos_left_6()
		baxter.move_left_arm(pos)
		
		pos = obj5.pos_right_4()
		baxter.move_right_arm(pos)		
				
		H = baxter.rightGripperpos()
		print "H = %s" % H 
		while  (H < 50):
			H = baxter.rightGripperpos()
			time.sleep(1)
		pos = obj5.pos_right_1()
		baxter.move_right_arm(pos)
		
##################### object 6 execution

		rospy.loginfo('Picking Object 6')
		

		pos = obj6.pos_right_1()
		baxter.move_right_arm(pos)
		
		pos = obj6.pos_left_1()
		baxter.move_left_arm(pos)
		
		pos = obj6.pos_left_2()
		baxter.move_left_arm(pos)
		
		time.sleep (0.5)
		baxter.openleftGripper()

		pos = obj6.pos_left_3()
		baxter.move_left_arm(pos)
		time.sleep (0.5)
		baxter.closeleftGripper()
		time.sleep (0.5)
		
		pos = obj6.pos_left_4()
		baxter.move_left_arm(pos)
		
		pos = obj6.pos_left_5()
		baxter.move_left_arm(pos)
		
		pos = obj6.pos_right_2()
		baxter.move_right_arm(pos)
		time.sleep (0.5)
		baxter.openrightGripper()
		
		pos = obj6.pos_right_3()
		baxter.move_right_arm(pos)
		
		baxter.closerightGripper()
		time.sleep (1)
		baxter.openleftGripper()
		time.sleep (0.5)
		
		pos = obj6.pos_left_6()
		baxter.move_left_arm(pos)
		
		pos = obj6.pos_right_4()
		baxter.move_right_arm(pos)		
				
		H = baxter.rightGripperpos()
		print "H = %s" % H 
		while  (H < 50):
			H = baxter.rightGripperpos()
			time.sleep(1)
		pos = obj6.pos_right_1()
		baxter.move_right_arm(pos)	
			
###################################################################################################
# doing opject 7 and object 8 and 9 Wrong location in the same speed		
	def case_3(self): 

		self.x = baxter.rightspeed
	
		print "Robot_speed = %s" % self.x 
		
		# Publishing joint_position_speed

		joint_speed = self.x
        	
		# Set joint position speed        	
		baxter.setjointspeed(self.x)
		
##################### object 7 execution

		rospy.loginfo('Picking Object 7')
		

		pos = obj7.pos_right_1()
		baxter.move_right_arm(pos)
		
		pos = obj7.pos_left_1()
		baxter.move_left_arm(pos)
		
		pos = obj7.pos_left_2()
		baxter.move_left_arm(pos)
		
		time.sleep (0.5)
		baxter.openleftGripper()

		pos = obj7.pos_left_3()
		baxter.move_left_arm(pos)
		time.sleep (0.5)
		baxter.closeleftGripper()
		time.sleep (0.5)
		
		pos = obj7.pos_left_4()
		baxter.move_left_arm(pos)
		
		pos = obj7.pos_left_5()
		baxter.move_left_arm(pos)
		
		pos = obj7.pos_right_2()
		baxter.move_right_arm(pos)
		time.sleep (0.5)
		baxter.openrightGripper()
		
		pos = obj7.pos_right_3()
		baxter.move_right_arm(pos)
		
		baxter.closerightGripper()
		time.sleep (1)
		baxter.openleftGripper()
		time.sleep (0.5)
		
		pos = obj7.pos_left_6()
		baxter.move_left_arm(pos)
		
		pos = wrong.pos_wronglocation()
		baxter.move_right_arm(pos)		
				
				
		H = baxter.rightGripperpos()
		print "H = %s" % H 
		while  (H < 50):
			H = baxter.rightGripperpos()
			time.sleep(1)
		pos = obj7.pos_right_1()
		baxter.move_right_arm(pos)
		
##################### object 8 execution

		rospy.loginfo('Picking Object 8')
		
		pos = obj8.pos_right_1()
		baxter.move_right_arm(pos)
		
		pos = obj8.pos_left_1()
		baxter.move_left_arm(pos)
		
		pos = obj8.pos_left_2()
		baxter.move_left_arm(pos)
		
		time.sleep (0.5)
		baxter.openleftGripper()

		pos = obj8.pos_left_3()
		baxter.move_left_arm(pos)
		time.sleep (0.5)
		baxter.closeleftGripper()
		time.sleep (0.5)
		
		pos = obj8.pos_left_4()
		baxter.move_left_arm(pos)
		
		pos = obj8.pos_left_5()
		baxter.move_left_arm(pos)
		
		pos = obj8.pos_right_2()
		baxter.move_right_arm(pos)
		time.sleep (0.5)
		baxter.openrightGripper()
		
		pos = obj8.pos_right_3()
		baxter.move_right_arm(pos)
		
		baxter.closerightGripper()
		time.sleep (1)
		baxter.openleftGripper()
		time.sleep (0.5)
		
		pos = obj8.pos_left_6()
		baxter.move_left_arm(pos)
		
		pos = wrong.pos_wronglocation()
		baxter.move_right_arm(pos)		
				
			
		H = baxter.rightGripperpos()
		print "H = %s" % H 
		while  (H < 50):
			H = baxter.rightGripperpos()
			time.sleep(1)
		pos = obj8.pos_right_1()
		baxter.move_right_arm(pos)
		
##################### object 9 execution

		rospy.loginfo('Picking Object 9')
		
		pos = obj9.pos_right_1()
		baxter.move_right_arm(pos)
		
		pos = obj9.pos_left_1()
		baxter.move_left_arm(pos)
		
		pos = obj9.pos_left_2()
		baxter.move_left_arm(pos)
		
		time.sleep (0.5)
		baxter.openleftGripper()

		pos = obj9.pos_left_3()
		baxter.move_left_arm(pos)
		time.sleep (0.5)
		baxter.closeleftGripper()
		time.sleep (0.5)
		
		pos = obj9.pos_left_4()
		baxter.move_left_arm(pos)
		
		pos = obj9.pos_left_5()
		baxter.move_left_arm(pos)
		
		pos = obj9.pos_right_2()
		baxter.move_right_arm(pos)
		time.sleep (0.5)
		baxter.openrightGripper()
		
		pos = obj9.pos_right_3()
		baxter.move_right_arm(pos)
		
		baxter.closerightGripper()
		time.sleep (1)
		baxter.openleftGripper()
		time.sleep (0.5)
		
		
		
		
		pos = obj9.pos_left_6()
		baxter.move_left_arm(pos)
		
				
		pos = wrong.pos_wronglocation()
		baxter.move_right_arm(pos)
				
		H = baxter.rightGripperpos()
		print "H = %s" % H 
		while  (H < 50):
			H = baxter.rightGripperpos()
			time.sleep(1)
		pos = obj9.pos_right_1()
		baxter.move_right_arm(pos)
		
		pos = obj9.pos_left_1()
		baxter.move_left_arm(pos)
		
		
#######################################################
	def case_4(self): # doing opject 7 and object 8 in the same speed

		self.x = baxter.rightspeed
	
		print "Robot_speed = %s" % self.x 
		
		# Publishing joint_position_speed		

		joint_speed = self.x

        	
		# Set joint position speed	
		baxter.setjointspeed(self.x)
		
		
############ object 1 execution
		rospy.loginfo('Picking Object 1')
		

		pos = obj1.pos_right_1()
		baxter.move_right_arm(pos)
		
		pos = obj1.pos_left_1()
		baxter.move_left_arm(pos)
		
		pos = obj1.pos_left_2()
		baxter.move_left_arm(pos)
		
		time.sleep (0.5)
		baxter.openleftGripper()

		pos = obj1.pos_left_3()
		baxter.move_left_arm(pos)
		time.sleep (0.5)
		baxter.closeleftGripper()
		time.sleep (0.5)
		
		pos = obj1.pos_left_4()
		baxter.move_left_arm(pos)
		
		pos = obj1.pos_left_5()
		baxter.move_left_arm(pos)
		
		pos = obj1.pos_right_2()
		baxter.move_right_arm(pos)
		time.sleep (0.5)
		baxter.openrightGripper()
		
		pos = obj1.pos_right_3()
		baxter.move_right_arm(pos)
		
		baxter.closerightGripper()
		time.sleep (1)
		baxter.openleftGripper()
		time.sleep (0.5)
		
		pos = obj1.pos_left_6()
		baxter.move_left_arm(pos)
		
		pos = wrong.pos_wronglocation()
		baxter.move_right_arm(pos)		
					
		H = baxter.rightGripperpos()
		print "H = %s" % H 
		while  (H < 50):
			H = baxter.rightGripperpos()
			time.sleep(1)
		pos = obj1.pos_right_1()
		baxter.move_right_arm(pos)
		
##################### object 2 execution

		rospy.loginfo('Picking Object 2')
		

		pos = obj2.pos_right_1()
		baxter.move_right_arm(pos)
		
		pos = obj2.pos_left_1()
		baxter.move_left_arm(pos)
		
		pos = obj2.pos_left_2()
		baxter.move_left_arm(pos)
		
		time.sleep (0.5)
		baxter.openleftGripper()

		pos = obj2.pos_left_3()
		baxter.move_left_arm(pos)
		time.sleep (0.5)
		baxter.closeleftGripper()
		time.sleep (0.5)
		
		pos = obj2.pos_left_4()
		baxter.move_left_arm(pos)
		
		pos = obj2.pos_left_5()
		baxter.move_left_arm(pos)
		
		pos = obj2.pos_right_2()
		baxter.move_right_arm(pos)
		time.sleep (0.5)
		baxter.openrightGripper()
		
		pos = obj2.pos_right_3()
		baxter.move_right_arm(pos)
		
		baxter.closerightGripper()
		time.sleep (1)
		baxter.openleftGripper()
		time.sleep (0.5)
		
		pos = obj2.pos_left_6()
		baxter.move_left_arm(pos)
		
		pos = wrong.pos_wronglocation()
		baxter.move_right_arm(pos)
		
		H = baxter.rightGripperpos()
		print "H = %s" % H 
		while  (H < 50):
			H = baxter.rightGripperpos()
			time.sleep(1)
		pos = obj2.pos_right_1()
		baxter.move_right_arm(pos)
		
##################### object 3 execution

		rospy.loginfo('Picking Object 3')
		

		pos = obj3.pos_right_1()
		baxter.move_right_arm(pos)
		
		pos = obj3.pos_left_1()
		baxter.move_left_arm(pos)
		
		pos = obj3.pos_left_2()
		baxter.move_left_arm(pos)
		
		time.sleep (0.5)
		baxter.openleftGripper()

		pos = obj3.pos_left_3()
		baxter.move_left_arm(pos)
		time.sleep (0.5)
		baxter.closeleftGripper()
		time.sleep (0.5)
		
		pos = obj3.pos_left_4()
		baxter.move_left_arm(pos)
		
		pos = obj3.pos_left_5()
		baxter.move_left_arm(pos)
		
		pos = obj3.pos_right_2()
		baxter.move_right_arm(pos)
		time.sleep (0.5)
		baxter.openrightGripper()
		
		pos = obj3.pos_right_3()
		baxter.move_right_arm(pos)
		
		baxter.closerightGripper()
		time.sleep (1)
		baxter.openleftGripper()
		time.sleep (0.5)
		
		pos = obj3.pos_left_6()
		baxter.move_left_arm(pos)
		
		pos = wrong.pos_wronglocation()
		baxter.move_right_arm(pos)
		
		H = baxter.rightGripperpos()
		print "H = %s" % H 
		while  (H < 50):

			H = baxter.rightGripperpos()
			time.sleep(1)
		pos = obj3.pos_right_1()
		baxter.move_right_arm(pos)	

#######################################################
	def case_5(self): # doing opject 7 and object 8 in the same speed

		self.x = baxter.rightspeed
	
		print "Robot_speed = %s" % self.x 
		
		# Publishing joint_position_speed		

		joint_speed = self.x

        	
		# Set joint position speed	
		baxter.setjointspeed(self.x)
		
##################### object 4 execution

		rospy.loginfo('Picking Object 4')
		
		pos = obj4.pos_right_1()
		baxter.move_right_arm(pos)
		
		pos = obj4.pos_left_1()
		baxter.move_left_arm(pos)
		
		pos = obj4.pos_left_2()
		baxter.move_left_arm(pos)
		
		time.sleep (0.5)
		baxter.openleftGripper()

		pos = obj4.pos_left_3()
		baxter.move_left_arm(pos)
		time.sleep (0.5)
		baxter.closeleftGripper()
		time.sleep (0.5)
		
		pos = obj4.pos_left_4()
		baxter.move_left_arm(pos)
		
		pos = obj4.pos_left_5()
		baxter.move_left_arm(pos)
		
		pos = obj4.pos_right_2()
		baxter.move_right_arm(pos)
		time.sleep (0.5)
		baxter.openrightGripper()
		
		pos = obj4.pos_right_3()
		baxter.move_right_arm(pos)
		
		baxter.closerightGripper()
		time.sleep (1)
		baxter.openleftGripper()
		time.sleep (0.5)
		
		pos = obj4.pos_left_6()
		baxter.move_left_arm(pos)
		
		pos = obj4.pos_right_4()
		baxter.move_right_arm(pos)		
				
		
		
		H = baxter.rightGripperpos()
		print "H = %s" % H 
		while  (H < 50):
			H = baxter.rightGripperpos()
			time.sleep(1)
		pos = obj4.pos_right_1()
		baxter.move_right_arm(pos)
		
		
##################### object 5 execution

		rospy.loginfo('Picking Object 5')
		
		pos = obj5.pos_right_1()
		baxter.move_right_arm(pos)
		
		pos = obj5.pos_left_1()
		baxter.move_left_arm(pos)
		
		pos = obj5.pos_left_2()
		baxter.move_left_arm(pos)
		
		time.sleep (0.5)
		baxter.openleftGripper()

		pos = obj5.pos_left_3()
		baxter.move_left_arm(pos)
		time.sleep (0.5)
		baxter.closeleftGripper()
		time.sleep (0.5)
		
		pos = obj5.pos_left_4()
		baxter.move_left_arm(pos)
		
		pos = obj5.pos_left_5()
		baxter.move_left_arm(pos)
		
		pos = obj5.pos_right_2()
		baxter.move_right_arm(pos)
		time.sleep (0.5)
		baxter.openrightGripper()
		
		pos = obj5.pos_right_3()
		baxter.move_right_arm(pos)
		
		baxter.closerightGripper()
		time.sleep (1)
		baxter.openleftGripper()
		time.sleep (0.5)
		
		pos = obj5.pos_left_6()
		baxter.move_left_arm(pos)
		
		pos = obj5.pos_right_4()
		baxter.move_right_arm(pos)		
				
		H = baxter.rightGripperpos()
		print "H = %s" % H 
		while  (H < 50):
			H = baxter.rightGripperpos()
			time.sleep(1)
		pos = obj5.pos_right_1()
		baxter.move_right_arm(pos)
		
##################### object 6 execution

		rospy.loginfo('Picking Object 6')
		
		pos = obj6.pos_right_1()
		baxter.move_right_arm(pos)
		
		pos = obj6.pos_left_1()
		baxter.move_left_arm(pos)
		
		pos = obj6.pos_left_2()
		baxter.move_left_arm(pos)
		
		time.sleep (0.5)
		baxter.openleftGripper()

		pos = obj6.pos_left_3()
		baxter.move_left_arm(pos)
		time.sleep (0.5)
		baxter.closeleftGripper()
		time.sleep (0.5)
		
		pos = obj6.pos_left_4()
		baxter.move_left_arm(pos)
		
		pos = obj6.pos_left_5()
		baxter.move_left_arm(pos)
		
		pos = obj6.pos_right_2()
		baxter.move_right_arm(pos)
		time.sleep (0.5)
		baxter.openrightGripper()
		
		pos = obj6.pos_right_3()
		baxter.move_right_arm(pos)
		
		baxter.closerightGripper()
		time.sleep (1)
		baxter.openleftGripper()
		time.sleep (0.5)
		
		pos = obj6.pos_left_6()
		baxter.move_left_arm(pos)
		
		pos = obj6.pos_right_4()
		baxter.move_right_arm(pos)		
						
		H = baxter.rightGripperpos()
		print "H = %s" % H 
		while  (H < 50):
			H = baxter.rightGripperpos()
			time.sleep(1)
		pos = obj6.pos_right_1()
		baxter.move_right_arm(pos)
		
		

###################################################################################################
# doing opject 7 and object 8 and 9 Wrong location in the same speed		
	def case_6(self): 

		self.x = baxter.rightspeed
	
		print "Robot_speed = %s" % self.x 
		
		# Publishing joint_position_speed

		joint_speed = self.x
        	
		# Set joint position speed        	
		baxter.setjointspeed(self.x)
		
##################### object 7 execution

		rospy.loginfo('Picking Object 7')
		
		pos = obj7.pos_right_1()
		baxter.move_right_arm(pos)
		
		pos = obj7.pos_left_1()
		baxter.move_left_arm(pos)
		
		pos = obj7.pos_left_2()
		baxter.move_left_arm(pos)
		
		time.sleep (0.5)
		baxter.openleftGripper()

		pos = obj7.pos_left_3()
		baxter.move_left_arm(pos)
		time.sleep (0.5)
		baxter.closeleftGripper()
		time.sleep (0.5)
		
		pos = obj7.pos_left_4()
		baxter.move_left_arm(pos)
		
		pos = obj7.pos_left_5()
		baxter.move_left_arm(pos)
		
		pos = obj7.pos_right_2()
		baxter.move_right_arm(pos)
		time.sleep (0.5)
		baxter.openrightGripper()
		
		pos = obj7.pos_right_3()
		baxter.move_right_arm(pos)
		
		baxter.openleftGripper()
		time.sleep (0.5)
		baxter.closerightGripper()
		time.sleep (0.5)
		
		
		pos = obj7.pos_left_6()
		baxter.move_left_arm(pos)
		
		pos = obj7.pos_right_4()
		baxter.move_right_arm(pos)		
				
				
		H = baxter.rightGripperpos()
		print "H = %s" % H 
		while  (H < 50):
			H = baxter.rightGripperpos()
			time.sleep(1)
		pos = obj7.pos_right_1()
		baxter.move_right_arm(pos)
		
##################### object 8 execution

		rospy.loginfo('Picking Object 8')
		
		pos = obj8.pos_right_1()
		baxter.move_right_arm(pos)
		
		pos = obj8.pos_left_1()
		baxter.move_left_arm(pos)
		
		pos = obj8.pos_left_2()
		baxter.move_left_arm(pos)
		
		time.sleep (0.5)
		baxter.openleftGripper()

		pos = obj8.pos_left_3()
		baxter.move_left_arm(pos)
		time.sleep (0.5)
		baxter.closeleftGripper()
		time.sleep (0.5)
		
		pos = obj8.pos_left_4()
		baxter.move_left_arm(pos)
		
		pos = obj8.pos_left_5()
		baxter.move_left_arm(pos)
		
		pos = obj8.pos_right_2()
		baxter.move_right_arm(pos)
		time.sleep (0.5)
		baxter.openrightGripper()
		
		pos = obj8.pos_right_3()
		baxter.move_right_arm(pos)
		
		baxter.openleftGripper()
		time.sleep (0.5)
		baxter.closerightGripper()
		time.sleep (0.5)
		
		
		pos = obj8.pos_left_6()
		baxter.move_left_arm(pos)
		
		pos = obj8.pos_right_4()
		baxter.move_right_arm(pos)		
				
			
		H = baxter.rightGripperpos()
		print "H = %s" % H 
		while  (H < 50):
			H = baxter.rightGripperpos()
			time.sleep(1)
		pos = obj8.pos_right_1()
		baxter.move_right_arm(pos)
		
##################### object 9 execution

		rospy.loginfo('Picking Object 9')
		
		pos = obj9.pos_right_1()
		baxter.move_right_arm(pos)
		
		pos = obj9.pos_left_1()
		baxter.move_left_arm(pos)
		
		pos = obj9.pos_left_2()
		baxter.move_left_arm(pos)
		
		time.sleep (0.5)
		baxter.openleftGripper()

		pos = obj9.pos_left_3()
		baxter.move_left_arm(pos)
		time.sleep (0.5)
		baxter.closeleftGripper()
		time.sleep (0.5)
		
		pos = obj9.pos_left_4()
		baxter.move_left_arm(pos)
		
		pos = obj9.pos_left_5()
		baxter.move_left_arm(pos)
		
		pos = obj9.pos_right_2()
		baxter.move_right_arm(pos)
		time.sleep (0.5)
		baxter.openrightGripper()
		
		pos = obj9.pos_right_3()
		baxter.move_right_arm(pos)
		
		time.sleep (0.5)
		baxter.openleftGripper()
		time.sleep (0.5)
		baxter.closerightGripper()
				
		pos = obj9.pos_left_6()
		baxter.move_left_arm(pos)
		
				
		pos = obj9.pos_right_4()
		baxter.move_right_arm(pos)
				
		H = baxter.rightGripperpos()
		print "H = %s" % H 
		while  (H < 50):
			H = baxter.rightGripperpos()
			time.sleep(1)
		pos = obj9.pos_right_1()
		baxter.move_right_arm(pos)
		
		pos = obj9.pos_left_1()
		baxter.move_left_arm(pos)	
		
		
		
#####################################################################################
# Objects 1, 2 and 3 in the right speed right location
	def case_7(self): 
		self.x = baxter.lowspeed
	
		print "Robot_speed = %s" % self.x 
		
		# Publishing joint_position_speed
		joint_speed = self.x

        	# Set joint position speed
		baxter.setjointspeed(self.x)
		
##################### object 1 execution
		rospy.loginfo('Picking Object 1')
		
		pos = obj1.pos_right_1()
		baxter.move_right_arm(pos)
		
		pos = obj1.pos_left_1()
		baxter.move_left_arm(pos)
		
		pos = obj1.pos_left_2()
		baxter.move_left_arm(pos)
		
		time.sleep (0.5)
		baxter.openleftGripper()

		pos = obj1.pos_left_3()
		baxter.move_left_arm(pos)
		time.sleep (0.5)
		baxter.closeleftGripper()
		time.sleep (0.5)
		
		pos = obj1.pos_left_4()
		baxter.move_left_arm(pos)
		
		pos = obj1.pos_left_5()
		baxter.move_left_arm(pos)
		
		pos = obj1.pos_right_2()
		baxter.move_right_arm(pos)
		time.sleep (0.5)
		baxter.openrightGripper()
		
		pos = obj1.pos_right_3()
		baxter.move_right_arm(pos)
		
		baxter.closerightGripper()
		time.sleep (1)
		baxter.openleftGripper()
		time.sleep (0.5)
		
		pos = obj1.pos_left_6()
		baxter.move_left_arm(pos)
		
		pos = obj1.pos_right_4()
		baxter.move_right_arm(pos)		
				
				
		H = baxter.rightGripperpos()
		print "H = %s" % H 
		while  (H < 50):
			H = baxter.rightGripperpos()
			time.sleep(1)
		pos = obj1.pos_right_1()
		baxter.move_right_arm(pos)
		
##################### object 2 execution

		rospy.loginfo('Picking Object 2')
		

		pos = obj2.pos_right_1()
		baxter.move_right_arm(pos)
		
		pos = obj2.pos_left_1()
		baxter.move_left_arm(pos)
		
		pos = obj2.pos_left_2()
		baxter.move_left_arm(pos)
		
		time.sleep (0.5)
		baxter.openleftGripper()

		pos = obj2.pos_left_3()
		baxter.move_left_arm(pos)
		time.sleep (0.5)
		baxter.closeleftGripper()
		time.sleep (0.5)
		
		pos = obj2.pos_left_4()
		baxter.move_left_arm(pos)
		
		pos = obj2.pos_left_5()
		baxter.move_left_arm(pos)
		
		pos = obj2.pos_right_2()
		baxter.move_right_arm(pos)
		time.sleep (0.5)
		baxter.openrightGripper()
		
		pos = obj2.pos_right_3()
		baxter.move_right_arm(pos)
		
		baxter.closerightGripper()
		time.sleep (1)
		baxter.openleftGripper()
		time.sleep (0.5)
		
		pos = obj2.pos_left_6()
		baxter.move_left_arm(pos)
		
		pos = obj2.pos_right_4()
		baxter.move_right_arm(pos)		
							
		H = baxter.rightGripperpos()
		print "H = %s" % H 
		while  (H < 50):
			H = baxter.rightGripperpos()
			time.sleep(1)
		pos = obj2.pos_right_1()
		baxter.move_right_arm(pos)
		


#######################################################################################		
# Objects 4, 5, and 6 low speed  and right location
	def case_8(self): 

		self.x = baxter.rightspeed
	
		print "Robot_speed = %s" % self.x
		 
		# Publishing joint_position_speed
		
		joint_speed = self.x

        	
		# Set joint position speed	
		baxter.setjointspeed(self.x)
		
#################### object 3 execution

		rospy.loginfo('Picking Object 3')
		
		pos = obj3.pos_right_1()
		baxter.move_right_arm(pos)
		
		pos = obj3.pos_left_1()
		baxter.move_left_arm(pos)
		
		pos = obj3.pos_left_2()
		baxter.move_left_arm(pos)
		
		time.sleep (0.5)
		baxter.openleftGripper()

		pos = obj3.pos_left_3()
		baxter.move_left_arm(pos)
		time.sleep (0.5)
		baxter.closeleftGripper()
		time.sleep (0.5)
		
		pos = obj3.pos_left_4()
		baxter.move_left_arm(pos)
		
		pos = obj3.pos_left_5()
		baxter.move_left_arm(pos)
		
		pos = obj3.pos_right_2()
		baxter.move_right_arm(pos)
		time.sleep (0.5)
		baxter.openrightGripper()
		
		pos = obj3.pos_right_3()
		baxter.move_right_arm(pos)
		
		baxter.closerightGripper()
		time.sleep (1)
		baxter.openleftGripper()
		time.sleep (0.5)
		
		pos = obj3.pos_left_6()
		baxter.move_left_arm(pos)
		
		pos = obj3.pos_right_4()
		baxter.move_right_arm(pos)		
				
				
		H = baxter.rightGripperpos()
		print "H = %s" % H 
		while  (H < 50):

			H = baxter.rightGripperpos()
			time.sleep(1)
		pos = obj3.pos_right_1()
		baxter.move_right_arm(pos)		
##################### object 4 execution

		rospy.loginfo('Picking Object 4')
		
		pos = obj4.pos_right_1()
		baxter.move_right_arm(pos)
		
		pos = obj4.pos_left_1()
		baxter.move_left_arm(pos)
		
		pos = obj4.pos_left_2()
		baxter.move_left_arm(pos)
		
		time.sleep (0.5)
		baxter.openleftGripper()

		pos = obj4.pos_left_3()
		baxter.move_left_arm(pos)
		time.sleep (0.5)
		baxter.closeleftGripper()
		time.sleep (0.5)
		
		pos = obj4.pos_left_4()
		baxter.move_left_arm(pos)
		
		pos = obj4.pos_left_5()
		baxter.move_left_arm(pos)
		
		pos = obj4.pos_right_2()
		baxter.move_right_arm(pos)
		time.sleep (0.5)
		baxter.openrightGripper()
		
		pos = obj4.pos_right_3()
		baxter.move_right_arm(pos)
		
		
		
		baxter.closerightGripper()
		time.sleep (1)
		baxter.openleftGripper()
		time.sleep (0.5)
		
		
		pos = obj4.pos_left_6()
		baxter.move_left_arm(pos)
		
		pos = obj4.pos_right_4()
		baxter.move_right_arm(pos)		
				
		#pos = obj4.pos_left_1()
		#baxter.move_left_arm(pos)
		
		H = baxter.rightGripperpos()
		print "H = %s" % H 
		while  (H < 50):
			H = baxter.rightGripperpos()
			time.sleep(1)
		pos = obj4.pos_right_1()
		baxter.move_right_arm(pos)
		
		
##################### object 5 execution

		rospy.loginfo('Picking Object 5')
		
		pos = obj5.pos_right_1()
		baxter.move_right_arm(pos)
		
		pos = obj5.pos_left_1()
		baxter.move_left_arm(pos)
		
		pos = obj5.pos_left_2()
		baxter.move_left_arm(pos)
		
		time.sleep (0.5)
		baxter.openleftGripper()

		pos = obj5.pos_left_3()
		baxter.move_left_arm(pos)
		time.sleep (0.5)
		baxter.closeleftGripper()
		time.sleep (0.5)
		
		pos = obj5.pos_left_4()
		baxter.move_left_arm(pos)
		
		pos = obj5.pos_left_5()
		baxter.move_left_arm(pos)
		
		pos = obj5.pos_right_2()
		baxter.move_right_arm(pos)
		time.sleep (0.5)
		baxter.openrightGripper()
		
		pos = obj5.pos_right_3()
		baxter.move_right_arm(pos)
		
		
		baxter.closerightGripper()
		time.sleep (1)
		baxter.openleftGripper()
		time.sleep (0.5)
		
		pos = obj5.pos_left_6()
		baxter.move_left_arm(pos)
		
		pos = obj5.pos_right_4()
		baxter.move_right_arm(pos)		
		
		H = baxter.rightGripperpos()
		print "H = %s" % H 
		while  (H < 50):
			H = baxter.rightGripperpos()
			time.sleep(1)
		pos = obj5.pos_right_1()
		baxter.move_right_arm(pos)
		
##################### object 6 execution

		rospy.loginfo('Picking Object 6')
		

		pos = obj6.pos_right_1()
		baxter.move_right_arm(pos)
		
		pos = obj6.pos_left_1()
		baxter.move_left_arm(pos)
		
		pos = obj6.pos_left_2()
		baxter.move_left_arm(pos)
		
		time.sleep (0.5)
		baxter.openleftGripper()

		pos = obj6.pos_left_3()
		baxter.move_left_arm(pos)
		time.sleep (0.5)
		baxter.closeleftGripper()
		time.sleep (0.5)
		
		pos = obj6.pos_left_4()
		baxter.move_left_arm(pos)
		
		pos = obj6.pos_left_5()
		baxter.move_left_arm(pos)
		
		pos = obj6.pos_right_2()
		baxter.move_right_arm(pos)
		time.sleep (0.5)
		baxter.openrightGripper()
		
		pos = obj6.pos_right_3()
		baxter.move_right_arm(pos)
		
		
		baxter.closerightGripper()
		time.sleep (1)
		baxter.openleftGripper()
		time.sleep (0.5)
		
		
		pos = obj6.pos_left_6()
		baxter.move_left_arm(pos)
		
		pos = obj6.pos_right_4()
		baxter.move_right_arm(pos)		
					
		H = baxter.rightGripperpos()
		print "H = %s" % H 
		while  (H < 50):
			H = baxter.rightGripperpos()
			time.sleep(1)
		pos = obj6.pos_right_1()
		baxter.move_right_arm(pos)
		
		
##################### object 7 execution

		rospy.loginfo('Picking Object 7')
		
		pos = obj7.pos_right_1()
		baxter.move_right_arm(pos)
		
		pos = obj7.pos_left_1()
		baxter.move_left_arm(pos)
		
		pos = obj7.pos_left_2()
		baxter.move_left_arm(pos)
		
		time.sleep (0.5)
		baxter.openleftGripper()

		pos = obj7.pos_left_3()
		baxter.move_left_arm(pos)
		time.sleep (0.5)
		baxter.closeleftGripper()
		time.sleep (0.5)
		
		pos = obj7.pos_left_4()
		baxter.move_left_arm(pos)
		
		pos = obj7.pos_left_5()
		baxter.move_left_arm(pos)
		
		pos = obj7.pos_right_2()
		baxter.move_right_arm(pos)
		time.sleep (0.5)
		baxter.openrightGripper()
		
		pos = obj7.pos_right_3()
		baxter.move_right_arm(pos)
		
		baxter.closerightGripper()
		time.sleep (1)
		baxter.openleftGripper()
		time.sleep (0.5)
		
		pos = obj7.pos_left_6()
		baxter.move_left_arm(pos)
		
		pos = obj7.pos_right_4()
		baxter.move_right_arm(pos)		
						
		H = baxter.rightGripperpos()
		print "H = %s" % H 
		while  (H < 50):
			H = baxter.rightGripperpos()
			time.sleep(1)
		pos = obj7.pos_right_1()
		baxter.move_right_arm(pos)
		
#################### object 8 execution

		rospy.loginfo('Picking Object 8')
		
		pos = obj8.pos_right_1()
		baxter.move_right_arm(pos)
		
		pos = obj8.pos_left_1()
		baxter.move_left_arm(pos)
		
		pos = obj8.pos_left_2()
		baxter.move_left_arm(pos)
		
		time.sleep (0.5)
		baxter.openleftGripper()

		pos = obj8.pos_left_3()
		baxter.move_left_arm(pos)
		time.sleep (0.5)
		baxter.closeleftGripper()
		time.sleep (0.5)
		
		pos = obj8.pos_left_4()
		baxter.move_left_arm(pos)
		
		pos = obj8.pos_left_5()
		baxter.move_left_arm(pos)
		
		pos = obj8.pos_right_2()
		baxter.move_right_arm(pos)
		time.sleep (0.5)
		baxter.openrightGripper()
		
		pos = obj8.pos_right_3()
		baxter.move_right_arm(pos)
		
		baxter.closerightGripper()
		time.sleep (1)
		baxter.openleftGripper()
		time.sleep (0.5)
		
		pos = obj8.pos_left_6()
		baxter.move_left_arm(pos)
		
		pos = obj8.pos_right_4()
		baxter.move_right_arm(pos)		
			
		H = baxter.rightGripperpos()
		print "H = %s" % H 
		while  (H < 50):
			H = baxter.rightGripperpos()
			time.sleep(1)
		pos = obj8.pos_right_1()
		baxter.move_right_arm(pos)
		
##################### object 9 execution

		rospy.loginfo('Picking Object 9')
		
		pos = obj9.pos_right_1()
		baxter.move_right_arm(pos)
		
		pos = obj9.pos_left_1()
		baxter.move_left_arm(pos)
		
		pos = obj9.pos_left_2()
		baxter.move_left_arm(pos)
		
		time.sleep (0.5)
		baxter.openleftGripper()

		pos = obj9.pos_left_3()
		baxter.move_left_arm(pos)
		time.sleep (0.5)
		baxter.closeleftGripper()
		time.sleep (0.5)
		
		pos = obj9.pos_left_4()
		baxter.move_left_arm(pos)
		
		pos = obj9.pos_left_5()
		baxter.move_left_arm(pos)
		
		pos = obj9.pos_right_2()
		baxter.move_right_arm(pos)
		time.sleep (0.5)
		baxter.openrightGripper()
		
		pos = obj9.pos_right_3()
		baxter.move_right_arm(pos)
		
		baxter.closerightGripper()
		time.sleep (1)
		baxter.openleftGripper()
		time.sleep (0.5)
				
		pos = obj9.pos_left_6()
		baxter.move_left_arm(pos)
						
		pos = obj9.pos_right_4()
		baxter.move_right_arm(pos)
				
		H = baxter.rightGripperpos()
		print "H = %s" % H 
		while  (H < 50):
			H = baxter.rightGripperpos()
			time.sleep(1)
		pos = obj9.pos_right_1()
		baxter.move_right_arm(pos)
		
		pos = obj9.pos_left_1()
		baxter.move_left_arm(pos)
