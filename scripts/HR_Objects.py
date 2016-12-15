#!/usr/bin/env python

import rospy
#import iodevices
import time
import numpy as np

#from HR_Baxter import *

#object1_position = []
#object1_orintation = []

#baxter = Baxter() # The object that controls Baxter.

class Object_1 (object):

	def __init__(self):
	
		pass
		
	def pos_left_1(self):
	
		self.obj1pos_left_1 = {'left_w0': 0.023438613032673857, 'left_w1': 0.5422619446385619, 'left_w2': -0.1542103112281453, 'left_e0': 0.025641403037236818, 'left_e1': 2.0869841140837013, 
				'left_s0': -0.6028141282271079, 'left_s1': -1.0983332426968844} # starting position
		
		pos = self.obj1pos_left_1
		return pos
		
	def pos_left_2(self):
		
		self.obj1pos_left_2 = {'left_w0': 0.43305544357954906, 'left_w1': -1.4655748165126299, 'left_w2': -0.031135685166902096, 'left_e0': 0.07983614331270086, 'left_e1': 2.300213215875499,
				'left_s0': -0.5526532491011273, 'left_s1': -0.4203458933019553} # Approching position
		
		pos = self.obj1pos_left_2
		return pos
		
		
	def pos_left_3(self):
		
		self.obj1pos_left_3 = {'left_w0': 0.5252793088898525, 'left_w1': -1.47079632679, 'left_w2': 0.15429523678428797, 'left_e0': -0.13545314280589882, 'left_e1': 2.00615005695897, 
				'left_s0': -0.37889488668961496, 'left_s1': -0.24280440935010184} # Grapping Position
		
		
		pos = self.obj1pos_left_3
		return pos
		
		
	def pos_left_4(self):	
	
		self.obj1pos_left_4 = {'left_w0': 0.030208535159625183, 'left_w1': -0.8363478363930252, 'left_w2': 0.36373502821424375, 'left_e0': -0.8734625802652298, 'left_e1': 2.3996112953608617,
				'left_s0': -0.01210502746093371, 'left_s1': -1.0507943874161991} # Retrieve position

		pos = self.obj1pos_left_4
		return pos
		
	def pos_left_5(self):	
		
		self.obj1pos_left_5 = {'left_w0': -0.6376322090176617, 'left_w1': 1.0462038197984644, 'left_w2': 1.523301337414949, 'left_e0': -0.7429664558680994, 'left_e1': 0.6347058820863878,
				'left_s0': -0.5296700155319776, 'left_s1': -0.40197589380628757} # Delivery Position Between Both Hands
				
		pos = self.obj1pos_left_5
		return pos
		
	def pos_left_6(self):		
		
		self.obj1pos_left_6 = {'left_w0': -0.6384608333457533, 'left_w1': 1.1972618503964958, 'left_w2': 1.5048228264589318, 'left_e0': -0.7440411498891979, 'left_e1': 0.727873696184806, 
				'left_s0': -0.3585482893499024, 'left_s1': -0.49392394879205}	# Retrieve position
				
		pos = self.obj1pos_left_6
		return pos
		
	def pos_right_1(self):	
		
		self.obj1pos_right_1 = {'right_s0': 0.2931809514821277, 'right_s1': -1.1213558080388566, 'right_w0': -0.09955710088747637, 'right_w1': 0.8141495099137936, 
				'right_w2':-0.0308224621762115, 'right_e0': 0.13051888865312958, 'right_e1': 1.8990737025269058} # starting position
		
		pos = self.obj1pos_right_1
		return pos
		#baxter.move_right_arm(self.pos_right_1)
		
		
	def pos_right_2(self):		
			
		self.obj1pos_right_2 = {'right_s0': -0.17493121792781005, 'right_s1': -0.31166030546827017, 'right_w0': 0.7748128327725197, 'right_w1': 0.9011410558640658, 
				'right_w2':-2.125995624336309,'right_e0': 1.0574632378514903, 'right_e1': 1.29658147864061} # Approching Position
		
		pos = self.obj1pos_right_2
		return pos
		
		
	def pos_right_3(self):	
	
		self.obj1pos_right_3 = {'right_s0': 0.07616332709637139, 'right_s1': -0.2631483744051039, 'right_w0': 0.9216211157848155, 'right_w1': 0.7096375759621596, 
				'right_w2':-2.2128838103055526, 'right_e0': 1.048949685355584, 'right_e1': 1.186878696302836} #Grapping POsition
		
		pos = self.obj1pos_right_3
		return pos
		
	def pos_right_4(self):
			
		self.obj1pos_right_4 = {'right_s0': -0.39251323143131944, 'right_s1': -0.06830141241068129, 'right_w0': 0.08211193470586144, 'right_w1': -0.6926103387907532, 
				'right_w2': 0.06536204249891854, 'right_e0': -0.14407952443468836, 'right_e1': 0.6600122188533681} # Delivery Position for Humman
		pos = self.obj1pos_right_4
		return pos
		
	
	
	
##############################	
	
class Object_2 (object):

	def __init__(self):
	
		pass
		
	def pos_left_1(self):
	
		self.obj2pos_left_1 = {'left_w0': 0.023438613032673857, 'left_w1': 0.5422619446385619, 'left_w2': -0.1542103112281453, 'left_e0': 0.025641403037236818, 'left_e1': 2.0869841140837013, 
				'left_s0': -0.6028141282271079, 'left_s1': -1.0983332426968844} # starting position
		
		pos = self.obj2pos_left_1
		return pos
		
	def pos_left_2(self):
		
		self.obj2pos_left_2 = {'left_w0': 1.2740027196859616, 'left_w1': -1.2781486240211868, 'left_w2': -0.03488515309336045, 'left_e0': -0.12046211560173307, 'left_e1': 1.772515806819572, 
				'left_s0': 0.10320449511245812, 'left_s1': -0.21973850097896458}# Approching position
		
		pos = self.obj2pos_left_2
		return pos
		
		
	def pos_left_3(self):
		
		self.obj2pos_left_3 = {'left_w0': 1.2083446121602037, 'left_w1': -1.2851765721412276, 'left_w2': -0.04958475890168548, 'left_e0': -0.1891417692054401, 'left_e1': 1.5243863041599652, 
				'left_s0': -0.014974736762836288, 'left_s1': -0.10646328673718722} # Grapping Position
		
		
		pos = self.obj2pos_left_3
		return pos
		
		
	def pos_left_4(self):	
	
		self.obj2pos_left_4 = {'left_w0': 0.5714667808033214, 'left_w1': -1.2357102982343444, 'left_w2': -0.1416305655844024, 'left_e0': -0.564364614844678, 'left_e1': 2.4735891430995274, 
				'left_s0': 0.17099738437738107, 'left_s1': -1.410193077894731} # Retrieve position

		pos = self.obj2pos_left_4
		return pos
		
	def pos_left_5(self):	
		
		self.obj2pos_left_5 = {'left_w0': -0.6376322090176617, 'left_w1': 1.0462038197984644, 'left_w2': 1.523301337414949, 'left_e0': -0.7429664558680994, 'left_e1': 0.6347058820863878,
				'left_s0': -0.5296700155319776, 'left_s1': -0.40197589380628757} # Delivery Position Between Both Hands

				
		pos = self.obj2pos_left_5
		return pos
		
	def pos_left_6(self):		
		
		self.obj2pos_left_6 = {'left_w0': -0.6384608333457533, 'left_w1': 1.1972618503964958, 'left_w2': 1.5048228264589318, 'left_e0': -0.7440411498891979, 'left_e1': 0.727873696184806, 
				'left_s0': -0.3585482893499024, 'left_s1': -0.49392394879205}	# Retrieve position
				
		pos = self.obj2pos_left_6
		return pos
		
	def pos_right_1(self):	
		
		self.obj2pos_right_1 = {'right_s0': 0.2931809514821277, 'right_s1': -1.1213558080388566, 'right_w0': -0.09955710088747637, 'right_w1': 0.8141495099137936, 
				'right_w2':-0.0308224621762115, 'right_e0': 0.13051888865312958, 'right_e1': 1.8990737025269058} # starting position
		
		pos = self.obj2pos_right_1
		return pos
		#baxter.move_right_arm(self.pos_right_1)
		
		
	def pos_right_2(self):		
			
		self.obj2pos_right_2 = {'right_s0': -0.17493121792781005, 'right_s1': -0.31166030546827017, 'right_w0': 0.7748128327725197, 'right_w1': 0.9011410558640658, 
				'right_w2':-2.125995624336309,'right_e0': 1.0574632378514903, 'right_e1': 1.29658147864061} # Approching Position
		
		pos = self.obj2pos_right_2
		return pos
		
		
	def pos_right_3(self):	
	
		self.obj2pos_right_3 = {'right_s0': 0.07616332709637139, 'right_s1': -0.2631483744051039, 'right_w0': 0.9216211157848155, 'right_w1': 0.7096375759621596, 
				'right_w2':-2.2128838103055526, 'right_e0': 1.048949685355584, 'right_e1': 1.186878696302836} #Grapping POsition
		
		pos = self.obj2pos_right_3
		return pos
		
	def pos_right_4(self):
			
		self.obj2pos_right_4 = {'right_s0': -0.39251323143131944, 'right_s1': -0.06830141241068129, 'right_w0': 0.08211193470586144, 'right_w1': -0.6926103387907532, 
				'right_w2': 0.06536204249891854, 'right_e0': -0.14407952443468836, 'right_e1': 0.6600122188533681} # Delivery Position for Humman
		pos = self.obj2pos_right_4
		return pos
		
		
##############################	
class Object_3 (object):

	def __init__(self):
	
		pass
		
	def pos_left_1(self):
	
		self.obj3pos_left_1 = {'left_w0': 0.023438613032673857, 'left_w1': 0.5422619446385619, 'left_w2': -0.1542103112281453, 'left_e0': 0.025641403037236818, 'left_e1': 2.0869841140837013, 
				'left_s0': -0.6028141282271079, 'left_s1': -1.0983332426968844} # starting position
		
		pos = self.obj3pos_left_1
		return pos
		
	def pos_left_2(self):
		
		self.obj3pos_left_2 = {'left_w0': 0.9047691031049888, 'left_w1': -1.33756122664493, 'left_w2': 0.02352892468401723, 'left_e0': -0.2067000755899355, 'left_e1': 1.736092692283982, 
				'left_s0': -0.14420191635751675, 'left_s1': -0.1358447281909912}# Approching position 
		
		pos = self.obj3pos_left_2
		return pos
		
		
	def pos_left_3(self):
		
		self.obj3pos_left_3 = {'left_w0': 0.9208715536015478, 'left_w1': -1.213761138425687, 'left_w2': 0.02516164995741133, 'left_e0': -0.3140105852528247, 'left_e1': 1.5036455862891798, 
				'left_s0': -0.17276474821901852, 'left_s1': -0.049509404802813566}# Grapping Position 
		
		
		pos = self.obj3pos_left_3
		return pos
		
		
	def pos_left_4(self):	
	
		self.obj3pos_left_4 = {'left_w0': 0.7892504411562152, 'left_w1': -0.7387123049414006, 'left_w2': -0.026211803014493308, 'left_e0': -0.9370901119033719, 'left_e1': 2.1195998604186452, 
				'left_s0': 0.27330979279275563, 'left_s1': -0.9477389064362504}# Retrieve position 

		pos = self.obj3pos_left_4
		return pos
		
	def pos_left_5(self):	
		
		self.obj3pos_left_5 = {'left_w0': -0.6376322090176617, 'left_w1': 1.0462038197984644, 'left_w2': 1.523301337414949, 'left_e0': -0.7429664558680994, 'left_e1': 0.6347058820863878,
				'left_s0': -0.5296700155319776, 'left_s1': -0.40197589380628757} # Delivery Position Between Both Hands

				
		pos = self.obj3pos_left_5
		return pos
		
	def pos_left_6(self):		
		
		self.obj3pos_left_6 = {'left_w0': -0.6384608333457533, 'left_w1': 1.1972618503964958, 'left_w2': 1.5048228264589318, 'left_e0': -0.7440411498891979, 'left_e1': 0.727873696184806, 
				'left_s0': -0.3585482893499024, 'left_s1': -0.49392394879205}	# Retrieve position
				
		pos = self.obj3pos_left_6
		return pos
		
	def pos_right_1(self):	
		
		self.obj3pos_right_1 = {'right_s0': 0.2931809514821277, 'right_s1': -1.1213558080388566, 'right_w0': -0.09955710088747637, 'right_w1': 0.8141495099137936, 
				'right_w2':-0.0308224621762115, 'right_e0': 0.13051888865312958, 'right_e1': 1.8990737025269058} # starting position
		
		pos = self.obj3pos_right_1
		return pos
		#baxter.move_right_arm(self.pos_right_1)
		
		
	def pos_right_2(self):		
			
		self.obj3pos_right_2 = {'right_s0': -0.17493121792781005, 'right_s1': -0.31166030546827017, 'right_w0': 0.7748128327725197, 'right_w1': 0.9011410558640658, 
				'right_w2':-2.125995624336309,'right_e0': 1.0574632378514903, 'right_e1': 1.29658147864061} # Approching Position
		
		pos = self.obj3pos_right_2
		return pos
		
		
	def pos_right_3(self):	
	
		self.obj3pos_right_3 = {'right_s0': 0.07616332709637139, 'right_s1': -0.2631483744051039, 'right_w0': 0.9216211157848155, 'right_w1': 0.7096375759621596, 
				'right_w2':-2.2128838103055526, 'right_e0': 1.048949685355584, 'right_e1': 1.186878696302836} #Grapping POsition
		
		pos = self.obj3pos_right_3
		return pos
		
	def pos_right_4(self):
			
		self.obj3pos_right_4 = {'right_s0': -0.39251323143131944, 'right_s1': -0.06830141241068129, 'right_w0': 0.08211193470586144, 'right_w1': -0.6926103387907532, 
				'right_w2': 0.06536204249891854, 'right_e0': -0.14407952443468836, 'right_e1': 0.6600122188533681} # Delivery Position for Humman
		pos = self.obj3pos_right_4
		return pos
		
		
#############################
class Object_4 (object):

	def __init__(self):
	
		pass
		
	def pos_left_1(self):
	
		self.obj4pos_left_1 = {'left_w0': 0.023438613032673857, 'left_w1': 0.5422619446385619, 'left_w2': -0.1542103112281453, 'left_e0': 0.025641403037236818, 'left_e1': 2.0869841140837013, 
				'left_s0': -0.6028141282271079, 'left_s1': -1.0983332426968844} # starting position
		
		pos = self.obj4pos_left_1
		return pos
		
	def pos_left_2(self):
		
		self.obj4pos_left_2 = {'left_w0': 0.8848536384221194, 'left_w1': -1.355905133386973, 'left_w2': -0.12867911812443242, 'left_e0': -0.01920264553733877, 'left_e1': 1.6578462635465265, 
				'left_s0': -0.08354812539802828, 'left_s1': -0.18638069856141248}# Approching position
		
		pos = self.obj4pos_left_2
		return pos
		
		
	def pos_left_3(self):
		
		self.obj4pos_left_3 = {'left_w0': 0.8674615454585178, 'left_w1': -1.283371573287007, 'left_w2': -0.16784028636050094, 'left_e0': -0.13401700175628548, 'left_e1': 1.4358164786188503, 
				'left_s0': -0.09392750363585889, 'left_s1': -0.09346846980210965}# Grapping Position
		
		
		pos = self.obj4pos_left_3
		return pos
		
		
	def pos_left_4(self):	
	
		self.obj4pos_left_4 = {'left_w0': 0.8095268452774398, 'left_w1': -0.7479069551183477, 'left_w2': -0.22819400473217083, 'left_e0': -0.8500514833554574, 'left_e1': 2.0412872328890628, 
				'left_s0': 0.4460593130129955, 'left_s1': -1.0236157588045343}# Retrieve position

		pos = self.obj4pos_left_4
		return pos
		
	def pos_left_5(self):	
		
		self.obj4pos_left_5 = {'left_w0': -0.6376322090176617, 'left_w1': 1.0462038197984644, 'left_w2': 1.523301337414949, 'left_e0': -0.7429664558680994, 'left_e1': 0.6347058820863878,
				'left_s0': -0.5296700155319776, 'left_s1': -0.40197589380628757} # Delivery Position Between Both Hands
				
		pos = self.obj4pos_left_5
		return pos
		
	def pos_left_6(self):		
		
		self.obj4pos_left_6 = {'left_w0': -0.6384608333457533, 'left_w1': 1.1972618503964958, 'left_w2': 1.5048228264589318, 'left_e0': -0.7440411498891979, 'left_e1': 0.727873696184806, 
				'left_s0': -0.3585482893499024, 'left_s1': -0.49392394879205}	# Retrieve position
				
		pos = self.obj4pos_left_6
		return pos
		
	def pos_right_1(self):	
		
		self.obj4pos_right_1 = {'right_s0': 0.2931809514821277, 'right_s1': -1.1213558080388566, 'right_w0': -0.09955710088747637, 'right_w1': 0.8141495099137936, 
				'right_w2':-0.0308224621762115, 'right_e0': 0.13051888865312958, 'right_e1': 1.8990737025269058} # starting position
		
		pos = self.obj4pos_right_1
		return pos
		#baxter.move_right_arm(self.pos_right_1)
		
		
	def pos_right_2(self):		
			
		self.obj4pos_right_2 = {'right_s0': -0.17493121792781005, 'right_s1': -0.31166030546827017, 'right_w0': 0.7748128327725197, 'right_w1': 0.9011410558640658, 
				'right_w2':-2.125995624336309,'right_e0': 1.0574632378514903, 'right_e1': 1.29658147864061} # Approching Position
		
		pos = self.obj4pos_right_2
		return pos
		
		
	def pos_right_3(self):	
	
		self.obj4pos_right_3 = {'right_s0': 0.07616332709637139, 'right_s1': -0.2631483744051039, 'right_w0': 0.9216211157848155, 'right_w1': 0.7096375759621596, 
				'right_w2':-2.2128838103055526, 'right_e0': 1.048949685355584, 'right_e1': 1.186878696302836} #Grapping POsition
		
		pos = self.obj4pos_right_3
		return pos
		
	def pos_right_4(self):
			
		self.obj4pos_right_4 = {'right_s0': -0.39251323143131944, 'right_s1': -0.06830141241068129, 'right_w0': 0.08211193470586144, 'right_w1': -0.6926103387907532, 
				'right_w2': 0.06536204249891854, 'right_e0': -0.14407952443468836, 'right_e1': 0.6600122188533681} # Delivery Position for Humman
		pos = self.obj4pos_right_4
		return pos
		
##############################
class Object_5 (object):

	def __init__(self):
	
		pass
		
	def pos_left_1(self):
	
		self.obj5pos_left_1 = {'left_w0': 0.023438613032673857, 'left_w1': 0.5422619446385619, 'left_w2': -0.1542103112281453, 'left_e0': 0.025641403037236818, 'left_e1': 2.0869841140837013, 
				'left_s0': -0.6028141282271079, 'left_s1': -1.0983332426968844} # starting position
		
		pos = self.obj5pos_left_1
		return pos
		
	def pos_left_2(self):
		
		self.obj5pos_left_2 = {'left_w0': 0.5839123256217865, 'left_w1': -1.2829610967465241, 'left_w2': -0.1672408276962361, 'left_e0': 0.11653030607019436, 'left_e1': 1.7529767015804383, 
				'left_s0': -0.407630645032198, 'left_s1': -0.269277193715831}# Approching position
		
		pos = self.obj5pos_left_2
		return pos
		
		
	def pos_left_3(self):
		
		self.obj5pos_left_3 = {'left_w0': 0.5083716696656047, 'left_w1': -1.29283978393563, 'left_w2': -0.2122122547562554, 'left_e0': 0.022825652734787837, 'left_e1': 1.4749234938472504, 
				'left_s0': -0.4139571496960502, 'left_s1': -0.15377644055822035}# Grapping Position
		
		
		pos = self.obj5pos_left_3
		return pos
		
		
	def pos_left_4(self):	
	
		self.obj5pos_left_4 = {'left_w0': 0.8095268452774398, 'left_w1': -0.7479069551183477, 'left_w2': -0.22819400473217083, 'left_e0': -0.8500514833554574, 'left_e1': 2.0412872328890628, 
				'left_s0': 0.4460593130129955, 'left_s1': -1.0236157588045343}# Retrieve position

		pos = self.obj5pos_left_4
		return pos
		
	def pos_left_5(self):	
		
		self.obj5pos_left_5 = {'left_w0': -0.6376322090176617, 'left_w1': 1.0462038197984644, 'left_w2': 1.523301337414949, 'left_e0': -0.7429664558680994, 'left_e1': 0.6347058820863878,
				'left_s0': -0.5296700155319776, 'left_s1': -0.40197589380628757} # Delivery Position Between Both Hands
				
		pos = self.obj5pos_left_5
		return pos
		
	def pos_left_6(self):		
		
		self.obj5pos_left_6 = {'left_w0': -0.6384608333457533, 'left_w1': 1.1972618503964958, 'left_w2': 1.5048228264589318, 'left_e0': -0.7440411498891979, 'left_e1': 0.727873696184806, 
				'left_s0': -0.3585482893499024, 'left_s1': -0.49392394879205}	# Retrieve position
				
		pos = self.obj5pos_left_6
		return pos
		
	def pos_right_1(self):	
		
		self.obj5pos_right_1 = {'right_s0': 0.2931809514821277, 'right_s1': -1.1213558080388566, 'right_w0': -0.09955710088747637, 'right_w1': 0.8141495099137936, 
				'right_w2':-0.0308224621762115, 'right_e0': 0.13051888865312958, 'right_e1': 1.8990737025269058} # starting position
		
		pos = self.obj5pos_right_1
		return pos
		#baxter.move_right_arm(self.pos_right_1)
		
		
	def pos_right_2(self):		
			
		self.obj5pos_right_2 = {'right_s0': -0.17493121792781005, 'right_s1': -0.31166030546827017, 'right_w0': 0.7748128327725197, 'right_w1': 0.9011410558640658, 
				'right_w2':-2.125995624336309,'right_e0': 1.0574632378514903, 'right_e1': 1.29658147864061} # Approching Position
		
		pos = self.obj5pos_right_2
		return pos
		
		
	def pos_right_3(self):	
	
		self.obj5pos_right_3 = {'right_s0': 0.07616332709637139, 'right_s1': -0.2631483744051039, 'right_w0': 0.9216211157848155, 'right_w1': 0.7096375759621596, 
				'right_w2':-2.2128838103055526, 'right_e0': 1.048949685355584, 'right_e1': 1.186878696302836} #Grapping POsition
		
		pos = self.obj5pos_right_3
		return pos
		
	def pos_right_4(self):
			
		self.obj5pos_right_4 = {'right_s0': -0.39251323143131944, 'right_s1': -0.06830141241068129, 'right_w0': 0.08211193470586144, 'right_w1': -0.6926103387907532, 
				'right_w2': 0.06536204249891854, 'right_e0': -0.14407952443468836, 'right_e1': 0.6600122188533681}# Delivery Position for Humman
		
		pos = self.obj5pos_right_4
		return pos

##############################
class Object_6 (object):

	def __init__(self):
	
		pass
		
	def pos_left_1(self):
	
		self.obj6pos_left_1 = {'left_w0': 0.023438613032673857, 'left_w1': 0.5422619446385619, 'left_w2': -0.1542103112281453, 'left_e0': 0.025641403037236818, 'left_e1': 2.0869841140837013, 
				'left_s0': -0.6028141282271079, 'left_s1': -1.0983332426968844} # starting position
		
		pos = self.obj6pos_left_1
		return pos
		
	def pos_left_2(self):
		
		self.obj6pos_left_2 = {'left_w0': 0.36181811387005264, 'left_w1': -1.321225791167287, 'left_w2': -0.13330238903610975, 'left_e0': 0.029815519178647044, 'left_e1': 1.52093960793949, 
				'left_s0': -0.5632193100109739, 'left_s1': -0.14495098379630658}
		
		pos = self.obj6pos_left_2
		return pos
		
		
	def pos_left_3(self):
		
		self.obj6pos_left_3 = {'left_w0': 0.33332495742877966, 'left_w1': -1.186521265912612, 'left_w2': -0.1797957092160352, 'left_e0': 0.009114667368346029, 'left_e1': 1.219911743794643, 					'left_s0': -0.5909005648852772, 'left_s1': -0.03797280352736106}
		
		
		pos = self.obj6pos_left_3
		return pos
		
		
	def pos_left_4(self):	
	
		self.obj6pos_left_4 = {'left_w0': 0.5706720948424095, 'left_w1': -0.788565734018272, 'left_w2': -0.3491727706847916, 'left_e0': -0.16334327369807858, 'left_e1': 1.842719729181918, 
				'left_s0': -0.4317411543042769, 'left_s1': -1.099135736574941}# Retrieve position

		pos = self.obj6pos_left_4
		return pos
		
	def pos_left_5(self):	
		
		self.obj6pos_left_5 = {'left_w0': -0.6376322090176617, 'left_w1': 1.0462038197984644, 'left_w2': 1.523301337414949, 'left_e0': -0.7429664558680994, 'left_e1': 0.6347058820863878,
				'left_s0': -0.5296700155319776, 'left_s1': -0.40197589380628757} # Delivery Position Between Both Hands
				
		pos = self.obj6pos_left_5
		return pos
		
	def pos_left_6(self):		
		
		self.obj6pos_left_6 = {'left_w0': -0.6384608333457533, 'left_w1': 1.1972618503964958, 'left_w2': 1.5048228264589318, 'left_e0': -0.7440411498891979, 'left_e1': 0.727873696184806, 
				'left_s0': -0.3585482893499024, 'left_s1': -0.49392394879205}	# Retrieve position
				
		pos = self.obj6pos_left_6
		return pos
		
	def pos_right_1(self):	
		
		self.obj6pos_right_1 = {'right_s0': 0.2931809514821277, 'right_s1': -1.1213558080388566, 'right_w0': -0.09955710088747637, 'right_w1': 0.8141495099137936, 
				'right_w2':-0.0308224621762115, 'right_e0': 0.13051888865312958, 'right_e1': 1.8990737025269058} # starting position
		
		pos = self.obj6pos_right_1
		return pos
		#baxter.move_right_arm(self.pos_right_1)
		
		
	def pos_right_2(self):		
			
		self.obj6pos_right_2 = {'right_s0': -0.17493121792781005, 'right_s1': -0.31166030546827017, 'right_w0': 0.7748128327725197, 'right_w1': 0.9011410558640658, 
				'right_w2':-2.125995624336309,'right_e0': 1.0574632378514903, 'right_e1': 1.29658147864061} # Approching Position
		
		pos = self.obj6pos_right_2
		return pos
		
		
	def pos_right_3(self):	
	
		self.obj6pos_right_3 = {'right_s0': 0.07616332709637139, 'right_s1': -0.2631483744051039, 'right_w0': 0.9216211157848155, 'right_w1': 0.7096375759621596, 
				'right_w2':-2.2128838103055526, 'right_e0': 1.048949685355584, 'right_e1': 1.186878696302836} #Grapping POsition
		
		pos = self.obj6pos_right_3
		return pos
		
	def pos_right_4(self):
			
		self.obj6pos_right_4 = {'right_s0': -0.39251323143131944, 'right_s1': -0.06830141241068129, 'right_w0': 0.08211193470586144, 'right_w1': -0.6926103387907532, 
				'right_w2': 0.06536204249891854, 'right_e0': -0.14407952443468836, 'right_e1': 0.6600122188533681} # Delivery Position for Humman
		pos = self.obj6pos_right_4
		return pos
		
##############################
class Object_7 (object):

	def __init__(self):
	
		pass
		
	def pos_left_1(self):
	
		self.obj7pos_left_1 = {'left_w0': 0.023438613032673857, 'left_w1': 0.5422619446385619, 'left_w2': -0.1542103112281453, 'left_e0': 0.025641403037236818, 'left_e1': 2.0869841140837013, 
				'left_s0': -0.6028141282271079, 'left_s1': -1.0983332426968844} # starting position
		
		pos = self.obj7pos_left_1
		return pos
		
	def pos_left_2(self):
		
		self.obj7pos_left_2 = {'left_w0': 0.24931723573374798, 'left_w1': -1.177492794716785, 'left_w2': -0.24535357622061438, 'left_e0': 0.08898353029583357, 'left_e1': 1.6662749178827772, 
				'left_s0': -0.3949155476288457, 'left_s1': -0.2779622571447491}# Approching position
		
		pos = self.obj7pos_left_2
		return pos
		
		
	def pos_left_3(self):
		
		self.obj7pos_left_3 = {'left_w0': 0.22449333593273949, 'left_w1': -1.2046054009318343, 'left_w2': -0.27633999692491235, 'left_e0': 0.09350237222637131, 'left_e1': 1.2609262241447146, 					'left_s0': -0.4040540537044609, 'left_s1': -0.056729987387397644}# Grapping Position
		
		
		pos = self.obj7pos_left_3
		return pos
		
		
	def pos_left_4(self):	
	
		self.obj7pos_left_4 = {'left_w0': 0.4740353353886425, 'left_w1': -0.6554169225415925, 'left_w2': -0.3586403470350301, 'left_e0': 0.013203114958928206, 'left_e1': 1.899453789610235, 
				'left_s0': -0.34412402021828814, 'left_s1': -1.2732042164561586}# Retrieve position

		pos = self.obj7pos_left_4
		return pos
		
	def pos_left_5(self):	
		
		self.obj7pos_left_5 = {'left_w0': -0.6376322090176617, 'left_w1': 1.0462038197984644, 'left_w2': 1.523301337414949, 'left_e0': -0.7429664558680994, 'left_e1': 0.6347058820863878,
				'left_s0': -0.5296700155319776, 'left_s1': -0.40197589380628757} # Delivery Position Between Both Hands
				
		pos = self.obj7pos_left_5
		return pos
		
	def pos_left_6(self):		
		
		self.obj7pos_left_6 = {'left_w0': -0.6384608333457533, 'left_w1': 1.1972618503964958, 'left_w2': 1.5048228264589318, 'left_e0': -0.7440411498891979, 'left_e1': 0.727873696184806, 
				'left_s0': -0.3585482893499024, 'left_s1': -0.49392394879205}	# Retrieve position
				
		pos = self.obj7pos_left_6
		return pos
		
	def pos_right_1(self):	
		
		self.obj7pos_right_1 = {'right_s0': 0.2931809514821277, 'right_s1': -1.1213558080388566, 'right_w0': -0.09955710088747637, 'right_w1': 0.8141495099137936, 
				'right_w2':-0.0308224621762115, 'right_e0': 0.13051888865312958, 'right_e1': 1.8990737025269058} # starting position
		
		pos = self.obj7pos_right_1
		return pos
		#baxter.move_right_arm(self.pos_right_1)
		
		
	def pos_right_2(self):		
			
		self.obj7pos_right_2 = {'right_s0': -0.17493121792781005, 'right_s1': -0.31166030546827017, 'right_w0': 0.7748128327725197, 'right_w1': 0.9011410558640658, 
				'right_w2':-2.125995624336309,'right_e0': 1.0574632378514903, 'right_e1': 1.29658147864061} # Approching Position
		
		pos = self.obj7pos_right_2
		return pos
		
		
	def pos_right_3(self):	
	
		self.obj7pos_right_3 = {'right_s0': 0.07616332709637139, 'right_s1': -0.2631483744051039, 'right_w0': 0.9216211157848155, 'right_w1': 0.7096375759621596, 
				'right_w2':-2.2128838103055526, 'right_e0': 1.048949685355584, 'right_e1': 1.186878696302836} #Grapping POsition
		
		pos = self.obj7pos_right_3
		return pos
		
	def pos_right_4(self):
			
		self.obj7pos_right_4 = {'right_s0': -0.39251323143131944, 'right_s1': -0.06830141241068129, 'right_w0': 0.08211193470586144, 'right_w1': -0.6926103387907532, 
				'right_w2': 0.06536204249891854, 'right_e0': -0.14407952443468836, 'right_e1': 0.6600122188533681} # Delivery Position for Humman
		pos = self.obj7pos_right_4
		return pos
		
		
##############################
class Object_8 (object):

	def __init__(self):
	
		pass
		
	def pos_left_1(self):
	
		self.obj8pos_left_1 = {'left_w0': 0.023438613032673857, 'left_w1': 0.5422619446385619, 'left_w2': -0.1542103112281453, 'left_e0': 0.025641403037236818, 'left_e1': 2.0869841140837013, 
				'left_s0': -0.6028141282271079, 'left_s1': -1.0983332426968844} # starting position
		
		pos = self.obj8pos_left_1
		return pos
		
	def pos_left_2(self):
		
		self.obj8pos_left_2 = {'left_w0': 0.09040624173976854, 'left_w1': -1.1765218720370567, 'left_w2': -0.14137733097099417, 'left_e0': 0.07304090786545718, 'left_e1': 1.5857626373309077, 
				'left_s0': -0.550579715389867, 'left_s1': -0.23972873349718749}# Approching position
		
		pos = self.obj8pos_left_2
		return pos
		
		
	def pos_left_3(self):
		
		self.obj8pos_left_3 = {'left_w0': 0.09024587153951046, 'left_w1': -1.0856958766243587, 'left_w2': -0.16357761904442972, 'left_e0': 0.08009957639761141, 'left_e1': 1.0526966786624756, 
				'left_s0': -0.5470804977617838, 'left_s1': 0.026863816077290884}# Grapping Position
		
		
		pos = self.obj8pos_left_3
		return pos
		
		
	def pos_left_4(self):	
	
		self.obj8pos_left_4 = {'left_w0': 0.09283491802097132, 'left_w1': -0.5384333734506862, 'left_w2': -0.1315993445380945, 'left_e0': 0.039595832976747874, 'left_e1': 1.6843148394325682, 
				'left_s0': -0.46525149497922214, 'left_s1': -1.1926697679178135}# Retrieve position

		pos = self.obj8pos_left_4
		return pos
		
	def pos_left_5(self):	
		
		self.obj8pos_left_5 = {'left_w0': -0.6376322090176617, 'left_w1': 1.0462038197984644, 'left_w2': 1.523301337414949, 'left_e0': -0.7429664558680994, 'left_e1': 0.6347058820863878,
				'left_s0': -0.5296700155319776, 'left_s1': -0.40197589380628757} # Delivery Position Between Both Hands
				
		pos = self.obj8pos_left_5
		return pos
		
	def pos_left_6(self):		
		
		self.obj8pos_left_6 = {'left_w0': -0.6384608333457533, 'left_w1': 1.1972618503964958, 'left_w2': 1.5048228264589318, 'left_e0': -0.7440411498891979, 'left_e1': 0.727873696184806, 
				'left_s0': -0.3585482893499024, 'left_s1': -0.49392394879205}	# Retrieve position
				
		pos = self.obj8pos_left_6
		return pos
		
	def pos_right_1(self):	
		
		self.obj8pos_right_1 = {'right_s0': 0.2931809514821277, 'right_s1': -1.1213558080388566, 'right_w0': -0.09955710088747637, 'right_w1': 0.8141495099137936, 
				'right_w2':-0.0308224621762115, 'right_e0': 0.13051888865312958, 'right_e1': 1.8990737025269058} # starting position
		
		pos = self.obj8pos_right_1
		return pos
		#baxter.move_right_arm(self.pos_right_1)
		
		
	def pos_right_2(self):		
			
		self.obj8pos_right_2 = {'right_s0': -0.17493121792781005, 'right_s1': -0.31166030546827017, 'right_w0': 0.7748128327725197, 'right_w1': 0.9011410558640658, 
				'right_w2':-2.125995624336309,'right_e0': 1.0574632378514903, 'right_e1': 1.29658147864061} # Approching Position
		
		pos = self.obj8pos_right_2
		return pos
		
		
	def pos_right_3(self):	
	
		self.obj8pos_right_3 = {'right_s0': 0.07616332709637139, 'right_s1': -0.2631483744051039, 'right_w0': 0.9216211157848155, 'right_w1': 0.7096375759621596, 
				'right_w2':-2.2128838103055526, 'right_e0': 1.048949685355584, 'right_e1': 1.186878696302836} #Grapping POsition
		
		pos = self.obj8pos_right_3
		return pos
		
	def pos_right_4(self):
			
		self.obj8pos_right_4 = {'right_s0': -0.39251323143131944, 'right_s1': -0.06830141241068129, 'right_w0': 0.08211193470586144, 'right_w1': -0.6926103387907532, 
				'right_w2': 0.06536204249891854, 'right_e0': -0.14407952443468836, 'right_e1': 0.6600122188533681} # Delivery Position for Humman
		pos = self.obj8pos_right_4
		return pos
		
##############################
class Object_9 (object):

	def __init__(self):
	
		pass
		
	def pos_left_1(self):
	
		self.obj9pos_left_1 = {'left_w0': 0.023438613032673857, 'left_w1': 0.5422619446385619, 'left_w2': -0.1542103112281453, 'left_e0': 0.025641403037236818, 'left_e1': 2.0869841140837013, 
				'left_s0': -0.6028141282271079, 'left_s1': -1.0983332426968844} # starting position
		
		pos = self.obj9pos_left_1
		return pos
		
	def pos_left_2(self):
		
		self.obj9pos_left_2 = {'left_w0': 0.01724632190574046, 'left_w1': -1.1501039752287507, 'left_w2': -0.14919614437250847, 'left_e0': 0.05716086955878823, 'left_e1': 1.4009098164395406, 
				'left_s0': -0.66307852004309, 'left_s1': -0.1633682846011038}# Approching position
		
		pos = self.obj9pos_left_2
		return pos
		
		
	def pos_left_3(self):
		
		self.obj9pos_left_3 = {'left_w0': -0.02837278224185663, 'left_w1': -0.8586532050920993, 'left_w2': -0.15186221142686152, 'left_e0': 0.10852386671335242, 'left_e1': 0.7416934915731288, 				'left_s0': -0.6688129361036457, 'left_s1': 0.1292311955827494}# Grapping Position

				
		pos = self.obj9pos_left_3
		return pos
		
		
	def pos_left_4(self):	
	
		self.obj9pos_left_4 = {'left_w0': 0.014212093034583113, 'left_w1': -0.5349858928886334, 'left_w2': -0.0499041575333847, 'left_e0': 0.01962187923978509, 'left_e1': 1.5938214898489833, 
				'left_s0': -0.6063510709939601, 'left_s1': -1.0764780175705386}# Retrieve position

		pos = self.obj9pos_left_4
		return pos
		
	def pos_left_5(self):	
		
		self.obj9pos_left_5 = {'left_w0': -0.6376322090176617, 'left_w1': 1.0462038197984644, 'left_w2': 1.523301337414949, 'left_e0': -0.7429664558680994, 'left_e1': 0.6347058820863878,
				'left_s0': -0.5296700155319776, 'left_s1': -0.40197589380628757} # Delivery Position Between Both Hands
				
		pos = self.obj9pos_left_5
		return pos
		
	def pos_left_6(self):		
		
		self.obj9pos_left_6 = {'left_w0': -0.6384608333457533, 'left_w1': 1.1972618503964958, 'left_w2': 1.5048228264589318, 'left_e0': -0.7440411498891979, 'left_e1': 0.727873696184806, 
				'left_s0': -0.3585482893499024, 'left_s1': -0.49392394879205}	# Retrieve position
				
		pos = self.obj9pos_left_6
		return pos
		
	def pos_right_1(self):	
		
		self.obj9pos_right_1 = {'right_s0': 0.2931809514821277, 'right_s1': -1.1213558080388566, 'right_w0': -0.09955710088747637, 'right_w1': 0.8141495099137936, 
				'right_w2':-0.0308224621762115, 'right_e0': 0.13051888865312958, 'right_e1': 1.8990737025269058} # starting position
		
		pos = self.obj9pos_right_1
		return pos
		#baxter.move_right_arm(self.pos_right_1)
		
		
	def pos_right_2(self):		
			
		self.obj9pos_right_2 = {'right_s0': -0.17493121792781005, 'right_s1': -0.31166030546827017, 'right_w0': 0.7748128327725197, 'right_w1': 0.9011410558640658, 
				'right_w2':-2.125995624336309,'right_e0': 1.0574632378514903, 'right_e1': 1.29658147864061} # Approching Position
		
		pos = self.obj9pos_right_2
		return pos
		
		
	def pos_right_3(self):	
	
		self.obj9pos_right_3 = {'right_s0': 0.07616332709637139, 'right_s1': -0.2631483744051039, 'right_w0': 0.9216211157848155, 'right_w1': 0.7096375759621596, 
				'right_w2':-2.2128838103055526, 'right_e0': 1.048949685355584, 'right_e1': 1.186878696302836} #Grapping POsition
		
		pos = self.obj9pos_right_3
		return pos
		
	def pos_right_4(self):
			
		self.obj9pos_right_4 = {'right_s0': -0.39251323143131944, 'right_s1': -0.06830141241068129, 'right_w0': 0.08211193470586144, 'right_w1': -0.6926103387907532, 
				'right_w2': 0.06536204249891854, 'right_e0': -0.14407952443468836, 'right_e1': 0.6600122188533681} # Delivery Position for Humman
		pos = self.obj9pos_right_4
		return pos
		
		
		
class Object_Wronglocation (object):

	def __init__(self):
	
		pass
		
	def pos_wronglocation(self):
	
		self.wronglocation = {'right_s0': 0.0776300898799447, 'right_s1': -0.10736732156748956, 'right_w0': -0.19732053682070702, 'right_w1': -1.171990224859082, 
				'right_w2': 0.19497018621025408, 'right_e0': -0.057769334273328476, 'right_e1': 1.1692752815540397} # Delivery wrong Position for Humman 
		
		pos = self.wronglocation
		
		return pos
		
		
