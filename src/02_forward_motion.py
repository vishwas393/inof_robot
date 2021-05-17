#!/usr/bin/env python

import RPi.GPIO as gpio
#import rospy
#from std_msgs.msg import String
import time

left        = 0
right       = 1
forward     = 0
backward    = 1
right_wheel = [24, 25]
left_wheel  = [8, 7]
wheel       = [left_wheel, right_wheel]
t_sleep     = 3

def stop_motion():
    gpio.output(right_wheel[forward], gpio.LOW)
    gpio.output(right_wheel[backward], gpio.LOW)
    gpio.output(left_wheel[forward], gpio.LOW)
    gpio.output(left_wheel[backward], gpio.LOW)


def setup_wheels():
    gpio.setmode(gpio.BCM)
    gpio.setup(left_wheel[forward], gpio.OUT)
    gpio.setup(left_wheel[backward], gpio.OUT)
    gpio.setup(right_wheel[forward], gpio.OUT)
    gpio.setup(right_wheel[backward], gpio.OUT)
    stop_motion()
    print ("Wheels are set up! \n")


def move_forward(sleep_time):
    gpio.output(left_wheel[forward], gpio.HIGH)
    gpio.output(left_wheel[backward], gpio.LOW)
    gpio.output(right_wheel[forward], gpio.HIGH)
    gpio.output(right_wheel[backward], gpio.LOW)
    print ("Moving Forward! \n")
    time.sleep(sleep_time)
    stop_motion()


def move_backward(sleep_time):
    gpio.output(left_wheel[forward], gpio.LOW)
    gpio.output(left_wheel[backward], gpio.HIGH)
    gpio.output(right_wheel[forward], gpio.LOW)
    gpio.output(right_wheel[backward], gpio.HIGH)
    print ("Moving Backward \n")
    time.sleep(sleep_time)
    stop_motion()

def callback(msg):
    if (msg.data == "forward"):
        move_forward()

    if (msg.data == "backward"):
        move_backward()

""" 
def listener():
    rospy.init_node('move_listener')
    setup_wheels()
    rospy.Subscribe("move_direction", String, callback)
    rospy.spin()
    gpio.cleanup()
"""

if __name__ == '__main__':
    #listener()
    setup_wheels()
    move_forward(t_sleep)
    move_backward(t_sleep)
    gpio.cleanup()
