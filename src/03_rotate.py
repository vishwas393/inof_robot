#!/usr/bin/env python

#import rospy
#from std_msgs.msg import String
import RPi.GPIO as gpio
import time

forward     = 0
backward    = 1
left_wheel  = [8, 7]
right_wheel = [24, 25]
wheels      = [left_wheel, right_wheel]
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


def rotate_clockwise(sleep_time):
    gpio.output(right_wheel[forward], gpio.LOW)
    gpio.output(right_wheel[backward], gpio.HIGH)
    gpio.output(left_wheel[forward], gpio.HIGH)
    gpio.output(left_wheel[backward], gpio.LOW)
    print ("Rotating Clockwise! \n")
    time.sleep(sleep_time)
    stop_motion()


def rotate_anticlockwise(sleep_time):
    gpio.output(right_wheel[forward], gpio.HIGH)
    gpio.output(right_wheel[backward], gpio.LOW)
    gpio.output(left_wheel[forward], gpio.LOW)
    gpio.output(left_wheel[backward], gpio.HIGH)
    print ("Rotating Anti-Clockwise! \n")
    time.sleep(sleep_time)
    stop_motion()
    

def callback(msg):
    if(msg.data == "clockwise"):
        rotate_clockwise()

    if(msg.data == "anticlockwise"):
        rotate_anticlockwise()

"""
def listener():
    rospy.init_node('direction_listener')
    setup_wheels()
    rospy.Subscribe("rotation_direction", String, callback)
    rospy.spin()
    gpio.cleanup()
"""

if __name__ == '__main__':
    #listener()
    setup_wheels()
    rotate_clockwise(t_sleep)
    time.sleep(1)
    rotate_anticlockwise(t_sleep)
    gpio.cleanup()
