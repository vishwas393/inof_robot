#!/usr/bin/env python

import RPi.GPIO as gpio
import rospy
import os
import time
from std_msgs.msg import String
from trivoyer.motions import *
from pyPS4Controller.controller import Controller

wheels = [[7,8],[25,24]]
node_name = "ps4controller_node"
topic_name = "ps4_controller_cmd"
buttons = [11, 5]

def shutdown_rpi():
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
	gpio.add_event_detect(buttons[0], gpio.FALLING, callback=shutdown_rpi, bouncetime=    800)
	gpio.add_event_detect(buttons[1], gpio.FALLING, callback=shutdown_rpi, bouncetime=    800)




class PS4Controller(Controller):
	


	def __init__(self, **kwargs):
		rospy.init_node(node_name)
		self.pub = rospy.Publisher(topic_name, String, queue_size=1)
		self.last_cmd = 1
		self.recv_cmd = String()
		rotate_by_degree(20, wheels, 0)
		rotate_by_degree(-40, wheels, 0)
		rotate_by_degree(20, wheels, 0)

		Controller.__init__(self, **kwargs)

	def publish_msg(self):
		self.pub.publish(self.recv_cmd)

	def on_R3_release(self):
		shutdown_rpi()
 
	def on_circle_release(self):
		rotate_by_degree(-10, wheels, 0)
		self.recv_cmd.data = "circle_release"
		self.publish_msg()
		return

	def on_square_release(self):
		rotate_by_degree(10, wheels, 0)
		self.recv_cmd.data = "square_release"
		self.publish_msg()
		return

	def on_triangle_release(self):
		self.last_cmd = 1
		move_forward(wheels)
		time.sleep(1)
		stop_motion(wheels)
		self.recv_cmd.data = "triangle_release"
		self.publish_msg()
		return

	def on_x_release(self):
		self.last_cmd = 0;
		move_backward(wheels)
		time.sleep(1)
		stop_motion(wheels)
		self.recv_cmd.data = "x_release"
		self.publish_msg()
		return

	def on_L2_press(self, val):
		self.last_cmd = 1
		move_forward(wheels)
		self.recv_cmd.data = "L2_press"
		self.publish_msg()
		return

	def on_R2_press(self, val):
		self.last_cmd = 0
		move_backward(wheels)
		self.recv_cmd.data = "R2_press"
		self.publish_msg()
		return

	def on_L3_right(self, val):
		rotate_by_degree(-5, wheels, 0)
		self.recv_cmd.data = "L3_right"
		self.publish_msg()

	def on_L3_left(self, val):
		rotate_by_degree(5, wheels, 0)
		self.recv_cmd.data = "L3_left"
		self.publish_msg()

	def on_L3_x_at_rest(self):
		move_forward(wheels) if (self.last_cmd%2) else move_backward(wheels)
		rotate_by_degree(0, wheels, 1)
		self.recv_cmd.data = "L3_x_rest"
		self.publish_msg()

	def on_L3_y_at_rest(self):
		move_forward(wheels) if (self.last_cmd%2) else move_backward(wheels)
		rotate_by_degree(0, wheels, 1)
		self.recv_cmd.data = "L3_y_rest"
		self.publish_msg()
		
	def on_L3_release(self):
		stop_motion(wheels)
		self.recv_cmd.data = "L3_release"
		self.publish_msg()
		return


if __name__ == "__main__":
	setup_wheels(wheels)
	setup_buttons()
	controller = PS4Controller(interface="/dev/input/js0", connecting_using_ds4drv=False)
	controller.listen()
	shutdown_rpi()

