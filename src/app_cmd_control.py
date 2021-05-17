#!/usr/bin/env python

import RPi.GPIO as gpio
import rospy
import time
import os
from std_msgs.msg import *
from inof_robot.motions import *

wheels = [[7,8],[25,24]]
buttons = [11, 5]
node_name = "cmd_control_motion_node"
topic = "control_cmd"
topics_arr = []

def shutdown_rpi(dummy):
	stop_motion(wheels)
	time.sleep(1)
	gpio.cleanup()
	nodes = os.popen("rosnode list").readlines()
	for i in range(len(nodes)):
		print(nodes[i])
		if nodes[i] != ("/" + node_name + "\n"):
			c_node = nodes[i].replace("\n", "")
			print(c_node)
			os.system("rosnode kill " + c_node)
	
	os.system("killall -9 roscore")
	os.system("shutdown -h now")
	

def setup_buttons():
	gpio.setup(buttons[0], gpio.IN, pull_up_down=gpio.PUD_UP)
	gpio.setup(buttons[1], gpio.IN, pull_up_down=gpio.PUD_UP)
	gpio.add_event_detect(buttons[0], gpio.FALLING, callback=shutdown_rpi, bouncetime=800)
	gpio.add_event_detect(buttons[1], gpio.FALLING, callback=shutdown_rpi, bouncetime=800)

class obstacle_avoidance_app():

	def distance_callback(self, msg):
		strarr = msg.data.split(':')
		if(strarr[0] == "R"):
			rotate_by_degree(int(strarr[1]), wheels, 0);

		if(strarr[0] == "F"):
			move_forward(wheels)
			time.sleep(int(strarr[1]))
			stop_motion(wheels)

		if(strarr[0] == "B"):
			move_backward(wheels)
			time.sleep(int(strarr[1]))
			stop_motion(wheels)
			
		

	def __init__(self):
		rospy.init_node(node_name)
		setup_wheels(wheels)
		setup_buttons()
		
		for i in range(4):
			rotate_by_degree(90, wheels, 0);
		
		self.sub = rospy.Subscriber(topic, String, self.distance_callback)
		rospy.spin()
		

if __name__ == '__main__':
	try:
		app = obstacle_avoidance_app()
	
	except rospy.ROSInterruptException or KeyboardInterrupt:
		print("Interrupt! \n")
		pass
