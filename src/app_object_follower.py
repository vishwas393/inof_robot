import cv2
import numpy as np


img_height = 600
img_width  = 600
img_center_x = img_width/2
img_center_y = img_height/2
min_area = 250
max_area = 120000


def callback(msg):
    if msg.trackObjArea != 0:
        if msg.trackObjArea > min_area and msg.trackObjArea < max_area:
            if msg.trackObjX > (img_center_x + (img_width/3)):
                rotate_by_degree(30, wheel, 1)
            elif msg.trackObjX < (img_center_x - (img_width/3)):
                rotate_by_degree(-30, wheel, 1)
            else:
                move_forward(wheel)
        elif msg.trackObjArea < min_area:
            #TODO: Keep rotating robot until it finds something
        elif msg.trackObjArea > max_area:
            stop_motion(wheel)
    else:
        #TODO: Keep rotating robot until it finds something



def app_object_follower():
    rospy.init_node('app_object_follower')
    rospy.Subscriber('object_location', object_location, callback)
    rospy.spin()
    

if __name__ = '__main__':
    app_object_follower()
