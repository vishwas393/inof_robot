import rospy
import cv2
import numpy as np
from inof_robot.msg import *

img_height = 600
img_width  = 600
img_center_x = img_width/2
img_center_y = img_height/2
min_area = 250
max_area = 120000


sel_clr = np.uint8([[[255,255,0]]])
hsv_sel_clr = cv2.cvtColor(sel_clr, cv2.COLOR_BGR2HSV)
hue = hsv_sel_clr[0][0][0]

upr_b = np.array([hue+10, 255, 255])
lwr_b = np.array([hue-10, 100, 100])
clr_msk = cv2.inRange(img_hsv, lwr_b, upr_b)


def detect_object():
    rospy.init_node('object_detection_node')
    pub = rospy.Publisher('object_location', ObjLocation)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        img = cv2.imread("greenball.jpeg")
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        contours, hierarchy= cv2.findContours(clr_msk, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours: 
            trackObjArea = 0
            trackObjX    = 0
            trackObjY    = 0
    
            x, y, w, h = cv2.boundingRect(c)
            boundRectArea = x*y

            if trackObjArea < boundRectArea:
                trackObjArea = boundRectArea
                trackObjX    = x + w/2
                trackObjY    = y + h/2
        
        location = ObjLocation()
        location.trackObjArea = trackObjArea
        location.trackObjX    = trackObjX 
        location.trackObjY    = trackObjY
        pub.publish(location)
        rate.sleep()

if __name__ == '__main__':
    detect_object()
