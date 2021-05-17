import RPi.GPIO as gpio
import time
import math

RPM = 49
radius = 6.8;
right = 0
left = 1
forward = 1
backward = 0

def stop_motion(wheels):
	gpio.setmode(gpio.BCM)
	gpio.output(wheels[right][forward], gpio.LOW)
	gpio.output(wheels[right][backward], gpio.LOW)
	gpio.output(wheels[left][forward], gpio.LOW)
	gpio.output(wheels[left][backward], gpio.LOW)
	#print("Stopping Motion! \n")


def setup_wheels(wheels):
	gpio.setmode(gpio.BCM)
	gpio.setup(wheels[left][forward], gpio.OUT)
	gpio.setup(wheels[left][backward], gpio.OUT)
	gpio.setup(wheels[right][forward], gpio.OUT)
	gpio.setup(wheels[right][backward], gpio.OUT)
	#print ("Wheels are set up! \n")


def move_forward(wheels):
	gpio.output(wheels[right][backward], gpio.LOW)
	gpio.output(wheels[right][forward], gpio.HIGH)
	gpio.output(wheels[left][forward], gpio.HIGH)
	gpio.output(wheels[left][backward], gpio.LOW)
	#print ("Moving Forward! \n")



def move_backward(wheels):
	gpio.output(wheels[right][backward], gpio.HIGH)
	gpio.output(wheels[right][forward], gpio.LOW)
	gpio.output(wheels[left][forward], gpio.LOW)
	gpio.output(wheels[left][backward], gpio.HIGH)
	#print ("Moving Backward \n")


def rotate_clockwise(wheels):
	gpio.output(wheels[right][backward], gpio.LOW)
	gpio.output(wheels[right][forward], gpio.HIGH)
	gpio.output(wheels[left][forward], gpio.LOW)
	gpio.output(wheels[left][backward], gpio.HIGH)
	#print ("Rotating Clockwise! \n")


def rotate_anticlockwise(wheels):
	gpio.output(wheels[right][backward], gpio.HIGH)
	gpio.output(wheels[right][forward], gpio.LOW)
	gpio.output(wheels[left][forward], gpio.HIGH)
	gpio.output(wheels[left][backward], gpio.LOW)
	#print ("Rotating Anti-Clockwise! \n")


def rotate_by_degree(angle, wheels, keep_prev_state):
	previous_states = [[gpio.LOW, gpio.LOW], [gpio.LOW, gpio.LOW]]
	previous_states[0][0] = gpio.HIGH if gpio.input(wheels[0][0]) else gpio.LOW
	previous_states[0][1] = gpio.HIGH if gpio.input(wheels[0][1]) else gpio.LOW
	previous_states[1][0] = gpio.HIGH if gpio.input(wheels[1][0]) else gpio.LOW
	previous_states[1][1] = gpio.HIGH if gpio.input(wheels[1][1]) else gpio.LOW
    

	stop_motion(wheels)
	if(angle < -180 or angle > 180):
		return

	#t = abs(angle/math.degrees(RPM*(2*math.pi)/60));
	t = abs(angle/(6*RPM));
	print("Angle:" + str(angle) + "   Time:" + str(t))
	if(angle >= 0):
		rotate_anticlockwise(wheels)
	else:
		rotate_clockwise(wheels)
    
	time.sleep(t)    
	stop_motion(wheels)

	if keep_prev_state:
		gpio.output(wheels[0][0], previous_states[0][0])
		gpio.output(wheels[0][1], previous_states[0][1])
		gpio.output(wheels[1][0], previous_states[1][0])
		gpio.output(wheels[1][1], previous_states[1][1])


