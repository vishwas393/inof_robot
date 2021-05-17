import RPi.GPIO as GPIO
import rospy

GPIO.setmode(GPIO.BCM)
rospy.init_node("parameter_set")
rospy.spin()
	
