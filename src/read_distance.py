#!/usr/bin/env python

import rospy
import time
import RPi.GPIO as gpio
from std_msgs.msg import Float32

pin_trig = 23
pin_echo = 22

class distance_measurement():

	def timer_callback(self, timer):
		s = time.time()
		raw_data = self.sense_raw_data()
		dist_val = self.calculate_distance(raw_data)
		if dist_val < 15:
			self.pub.publish(dist_val)




	def setup_sensor(self):
		gpio.setmode(gpio.BCM)
		gpio.setup(pin_trig, gpio.OUT)
		gpio.setup(pin_echo, gpio.IN)
		gpio.output(pin_trig, gpio.LOW)
		time.sleep(2)

	def sense_raw_data(self):
		s_time = time.time()
		e_time = s_time

		gpio.output(pin_trig, gpio.LOW)
		gpio.output(pin_trig, gpio.HIGH)
		time.sleep(0.00001)
		gpio.output(pin_trig, gpio.LOW)

		while gpio.input(pin_echo) == 0:
			s_time = time.time()

		while gpio.input(pin_echo) == 1:
			e_time = time.time()

		return (e_time-s_time)

	# Ret: distance in centimeter value
	def calculate_distance(self, raw):
		return (raw*34300/2)


	def __init__(self):
		rospy.init_node('distance_sensor')
		self.setup_sensor()
		self.pub = rospy.Publisher('distance_val', Float32, queue_size=1)
		self.timer = rospy.Timer(rospy.Duration(0, 500000000), self.timer_callback)
		rospy.spin()



if __name__ == '__main__':
	try:
		application = distance_measurement()
	except KeyboardInterrupt:
		#gpio.cleanup(pin_trig)
		#gpio.cleanup(pin_echo)
		gpio.cleanup()
		ptint("KeyboardInterrupt! \n")	

	print("Exiting! \n")
