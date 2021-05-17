#!/usr/bin/env python

import RPi.GPIO as gpio
import rospy
import time
from std_msgs.msg import Float32
import qrcode
import ../lib/motions.py

wheels = [24, 25, 26]
qrcode local_qrcode
deliveries = ["Ofice-1", "Office-3"]

def qr_callback(msg):
    if(msg.qr_text in deliveries):
        local_qrcode = msg;
        #TODO: Goto Given Position and come back 
    else:
        local_qrcode.qr_detect = False;


def distance_callback(msg):
    if(msg.data <= 15):  
        stop_motion(wheels)
    else:
        move_forward(wheels)


def run_application():
    rospy.init('app_obstacle_avoidance');
    setup_wheels(wheels)
    rospy.Subscriber("distance_val", Float32, distance_callback)
    rospy.Subscriber("QRCode_available", qrcode, qr_callback)
    move_forward(wheels)
    rospy.spin()


if __name__ == '__main__':
    run_application()
