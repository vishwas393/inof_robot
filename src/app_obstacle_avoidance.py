#!/usr/bin/env python

import RPi.GPIO as gpio
import rospy
import time
import os
from std_msgs.msg import Float32
from inof_robot.motions import *

wheels = [[7,8],[25,24]]
buttons = [11, 5]
node_name = "app_obstacle_avoidance"
topic = "distance_val"
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
		if(msg.data <= 15): 
			rotate_by_degree(175, wheels, 1)
		

	def __init__(self):
		rospy.init_node(node_name)
		setup_wheels(wheels)
		setup_buttons()
		rate = rospy.Rate(1)
		cnt = 0
		while  cnt<10:
			print("...")
			t_arr = rospy.get_published_topics()
			for i in range(len(t_arr)):
				print(t_arr[i][0])
				topics_arr.append(t_arr[i][0])
			
			if ("/"+topic) in topics_arr:
				cnt = 0
				break

			cnt = cnt +1
			rate.sleep()
		
		if cnt == 0:
			print("topic_found! \n")
			self.sub = rospy.Subscriber(topic, Float32, self.distance_callback)
			move_forward(wheels)
			rospy.spin()
		else:
			return




if __name__ == '__main__':
	try:
		app = obstacle_avoidance_app()
	
	except rospy.ROSInterruptException or KeyboardInterrupt:
		print("ROS Interrupt! \n")
		pass
