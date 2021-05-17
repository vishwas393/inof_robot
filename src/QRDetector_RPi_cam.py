#!/usr/bin/env python

import cv2
import rospy
import std_msgs.msg
import sensor_msgs.msg
import qrcode
from cv_bridge import CvBridge
from picamera import PiCamera


def QRCodeDetectorNDecoder():
    rospy.init_node("QRcode_detector_decoder")
    #img = cv2.imread('/home/vishwas/ros/src/inof_robot/img/office-1.png')
    pub = rospy.Publisher('QRCode_available', qrcode, queue_size=1)
    detector = cv2.QRCodeDetector()
    
    txt, points, _ = detector.detectAndDecode(img)
    if points is not NONE:
        qrcode msg
        msg.qr_detected = True
        msg.qr_text     = txt

        rospy.publish(msg)
        rospy.loginfo(("Found QR Code: Text: " + txt))


if __name__ == '__main__':
    QRCodeDetectorNDecoder()
