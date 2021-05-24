#!/usr/bin/env python

from picamera.array import PiRGBArray
from picamera import PiCamera
import rospy
import cv2
import numpy as np
from trivoyer.msg import *

img_height = 480
img_width  = 640
img_center_x = img_width/2
img_center_y = img_height/2
min_area = 250
max_area = 100000


sel_clr = np.uint8([[[0,255,255]]])							#Yellow Colour
hsv_sel_clr = cv2.cvtColor(sel_clr, cv2.COLOR_BGR2HSV)
hue = hsv_sel_clr[0][0][0]

upr_b = np.array([hue+10, 255, 255])
lwr_b = np.array([hue-10, 100, 100])



def detect_object():
	#Set up ros
	rospy.init_node('object_detection_node')
	pub = rospy.Publisher('object_location', ObjLocation, queue_size=1)
	rate = rospy.Rate(2)

	#Set up Camera
	cam = PiCamera()
	cam.resolution = (img_width, img_height)
	cam.framerate = 32
	capture = PiRGBArray(cam, size=(img_width, img_height))
	
	print("Entering for loop!")

	while not rospy.is_shutdown():
		for frame in cam.capture_continuous(capture, format="bgr", use_video_port=True):
			img = frame.array

			img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
			clr_msk = cv2.inRange(img_hsv, lwr_b, upr_b)
			
			#cv2.imshow("Image", clr_msk)	
			#key = cv2.waitKey(1) & 0x01
			
			img2, contours, hierarchy= cv2.findContours(clr_msk, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

			trackObjArea = 0
			trackObjX    = 0
			trackObjY    = 0
			
			for c in contours: 
				x, y, w, h = cv2.boundingRect(c)
				boundRectArea = w*h

				if trackObjArea < boundRectArea:
					trackObjArea = boundRectArea
					trackObjX    = x + w/2
					trackObjY    = y + h/2

			location = ObjLocation()
			location.trackObjArea = float(trackObjArea)
			location.trackObjX    = float(trackObjX)
			location.trackObjY    = float(trackObjY)
			pub.publish(location)
			capture.truncate(0)
			rate.sleep()

if __name__ == '__main__':
    detect_object()
