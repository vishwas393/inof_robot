#!/usr/bin/env python

import rospy
import numpy as np
from trivoyer.msg import *
from trivoyer.motions import *
import RPi.GPIO as gpio
import time
import os

img_height = 480
img_width  = 640
img_center_x = img_width/2
img_center_y = img_height/2
min_area = 250
max_area = 100000

wheels = [[7,8],[25,24]]
buttons = [11, 5]
node_name = "app_object_follower"
state = False


def shutdown_rpi(dummy):
	stop_motion(wheels)
	#time.sleep(1)
	gpio.cleanup()
	nodes = os.popen("rosnode list").readlines()
	for i in range(len(nodes)):
		#print(nodes[i])
		if nodes[i] != ("/" + node_name + "\n"):
			c_node = nodes[i].replace("\n", "")
			#print(c_node)
			os.system("rosnode kill " + c_node)
	os.system("killall -9 roscore")
	os.system("shutdown -h now")
	

 
def setup_buttons():
	gpio.setmode(gpio.BCM)
	gpio.setup(buttons[0], gpio.IN, pull_up_down=gpio.PUD_UP)
	gpio.setup(buttons[1], gpio.IN, pull_up_down=gpio.PUD_UP)
	gpio.add_event_detect(buttons[0], gpio.FALLING, callback=shutdown_rpi, bouncetime=800)
	gpio.add_event_detect(buttons[1], gpio.FALLING, callback=shutdown_rpi, bouncetime=800)


class app_object_follower():
	
	def callback(self, msg):
		if msg.trackObjArea != 0:
			#print(msg.trackObjArea)
			if msg.trackObjArea > min_area and msg.trackObjArea < max_area:
				
				if msg.trackObjX > (img_center_x + (img_width/4)):
					rotate_by_degree(-30, wheels, 1)
					print("Rotating by -30 degree!")
				elif msg.trackObjX < (img_center_x - (img_width/4)):
					rotate_by_degree(30, wheels, 1)
					print("Rotating by 30 degree!")
				else:
					move_forward(wheels)
					print("Moving Forward!")

			elif msg.trackObjArea < min_area:
				print("Searching for object!")
				rotate_by_degree(30, wheels, 1)

			elif msg.trackObjArea > max_area:
				print("Stopping Motion!")
				stop_motion(wheels)
			
		else:
			print("Searching for object!")
			stop_motion(wheels)
			rotate_by_degree(30, wheels, 1)
		



	def __init__(self):
		setup_wheels(wheels)
		setup_buttons()
		rospy.init_node(node_name)
		self.sub = rospy.Subscriber('object_location', ObjLocation, self.callback)
		rospy.spin()
    

if __name__ == '__main__':
	application = app_object_follower()
