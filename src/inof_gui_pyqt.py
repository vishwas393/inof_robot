#!/usr/bin/env python

import rospy
import sys
import math
from gazebo_msgs.msg import ModelStates, ModelState
from inof_robot.srv import path_points
from inof_robot.msg import *
from geometry_msgs.msg import Quaternion
from PyQt5 import QtCore, QtGui, QtWidgets

arr = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0], [0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0], [0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0], [0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0], [0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0], [0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0], [0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0], [0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0], [0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0], [0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0], [0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0], [0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0], [0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0], [0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0], [0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0], [0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0], [0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0], [0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0], [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0], [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0], [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0], [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0]]



def pathTextBoxStringMaker(plist):
    row = []
    i = 0

    pathstr = "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n""<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n""p, li { white-space: pre-wrap; }\n""</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
    tmpstr = ""
    
    for line in arr:
        j = 0
        tmpstr = "<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:10pt;\">"
        for e in line:
            tpoint = Pose()
            tpoint.x = i
            tpoint.y = j
            tpoint.t = 0
            if (plist.count(tpoint)):
                tmpstr += "</span><span style=\" font-size:10pt; color:#33cc33;\">"
            tmpstr += str(e)
            tmpstr += "   "

            if (plist.count(tpoint)):
                tmpstr += "</span><span style=\" font-size:10pt;\">"
            j += 1
        tmpstr += "</span></p>"
        i += 1
        row.append(tmpstr)

    for s in range(len(row)-1, -1, -1):
        pathstr += row[s]


    pathstr += "</body><html>"
    return pathstr






def mapTextBoxMaker():
    row = []
    pathstr = "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n""<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n""p, li { white-space: pre-wrap; }\n""</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
    
    i = 0
    tmpstr = ""
    for line in arr:
        j = 0
        tmpstr = "<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:10pt;\">"
        for e in line:
            tmpstr += str(e)
            tmpstr += "   "
            j += 1
        tmpstr += "</span></p>"
        i += 1
        row.append(tmpstr)
    
    for s in range(len(row)-1, -1, -1):
        pathstr += row[s]


    pathstr += "</body><html>"
    return pathstr



def ToEulerAngles(q):
    x = 0
    y = 0
    z = 0

    #roll (x-axis rotation)
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    x = math.atan2(sinr_cosp, cosr_cosp)

    #pitch (y-axis rotation)
    sinp = 2 * (q.w * q.y - q.z * q.x)
    if (abs(sinp) >= 1):
        y = math.copysign(math.pi / 2, sinp)       # use 90 degrees if out of range
    else:
        y = math.asin(sinp)

    #yaw (z-axis rotation)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    z = math.atan2(siny_cosp, cosy_cosp)

    return z



class Ui_Form(object):
    def SendGoalPoint(self, x, y):
        p = Pose()
        p.x = x
        p.y = y
        p.t = 0

        self.pub.publish(p)
        self.currLocSub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.updateCurrLocation)

    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(1200, 800)
        self.line = QtWidgets.QFrame(Form)
        self.line.setGeometry(QtCore.QRect(20, 130, 1151, 20))
        self.line.setFrameShape(QtWidgets.QFrame.HLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.PathPreview = QtWidgets.QLabel(Form)
        self.PathPreview.setGeometry(QtCore.QRect(290, 200, 881, 581))
        self.PathPreview.setObjectName("PathPreview")
        self.HorizontalLineBig = QtWidgets.QLabel(Form)
        self.HorizontalLineBig.setGeometry(QtCore.QRect(420, 10, 341, 131))
        font = QtGui.QFont()
        font.setPointSize(57)
        self.HorizontalLineBig.setFont(font)
        self.HorizontalLineBig.setObjectName("HorizontalLineBig")
        self.DesXspinBox = QtWidgets.QSpinBox(Form)
        self.DesXspinBox.setGeometry(QtCore.QRect(70, 193, 71, 26))
        self.DesXspinBox.setObjectName("DesXspinBox")
        self.DesY = QtWidgets.QLabel(Form)
        self.DesY.setGeometry(QtCore.QRect(20, 223, 41, 31))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.DesY.setFont(font)
        self.DesY.setObjectName("DesY")
        self.CurrX = QtWidgets.QLabel(Form)
        self.CurrX.setGeometry(QtCore.QRect(20, 454, 41, 31))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.CurrX.setFont(font)
        self.CurrX.setObjectName("CurrX")
        self.DestinationCoordLabel = QtWidgets.QLabel(Form)
        self.DestinationCoordLabel.setGeometry(QtCore.QRect(20, 166, 211, 17))
        font = QtGui.QFont()
        font.setPointSize(13)
        font.setBold(True)
        font.setWeight(75)
        self.DestinationCoordLabel.setFont(font)
        self.DestinationCoordLabel.setObjectName("DestinationCoordLabel")
        self.VerticalLine = QtWidgets.QFrame(Form)
        self.VerticalLine.setGeometry(QtCore.QRect(260, 150, 21, 631))
        self.VerticalLine.setFrameShape(QtWidgets.QFrame.VLine)
        self.VerticalLine.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.VerticalLine.setObjectName("VerticalLine")
        self.DesYspinBox = QtWidgets.QSpinBox(Form)
        self.DesYspinBox.setGeometry(QtCore.QRect(70, 226, 71, 26))
        self.DesYspinBox.setObjectName("DesYspinBox")
        self.DesX = QtWidgets.QLabel(Form)
        self.DesX.setGeometry(QtCore.QRect(20, 190, 41, 31))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.DesX.setFont(font)
        self.DesX.setObjectName("DesX")
        self.CurrY = QtWidgets.QLabel(Form)
        self.CurrY.setGeometry(QtCore.QRect(20, 487, 41, 31))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.CurrY.setFont(font)
        self.CurrY.setObjectName("CurrY")
        self.WarningWrongDes = QtWidgets.QLabel(Form)
        self.WarningWrongDes.setGeometry(QtCore.QRect(30, 380, 67, 17))
        self.WarningWrongDes.setText("")
        self.WarningWrongDes.setObjectName("WarningWrongDes")
        self.CurrTh = QtWidgets.QLabel(Form)
        self.CurrTh.setGeometry(QtCore.QRect(20, 520, 41, 31))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.CurrTh.setFont(font)
        self.CurrTh.setObjectName("CurrTh")
        self.AngleLabel = QtWidgets.QLabel(Form)
        self.AngleLabel.setGeometry(QtCore.QRect(20, 290, 191, 41))
        font = QtGui.QFont()
        font.setPointSize(9)
        font.setItalic(True)
        self.AngleLabel.setFont(font)
        self.AngleLabel.setWordWrap(True)
        self.AngleLabel.setObjectName("AngleLabel")
        self.DesTh = QtWidgets.QLabel(Form)
        self.DesTh.setGeometry(QtCore.QRect(20, 256, 41, 31))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.DesTh.setFont(font)
        self.DesTh.setObjectName("DesTh")
        self.CurruntCoordLabel = QtWidgets.QLabel(Form)
        self.CurruntCoordLabel.setGeometry(QtCore.QRect(20, 430, 211, 17))
        font = QtGui.QFont()
        font.setPointSize(13)
        font.setBold(True)
        font.setWeight(75)
        self.CurruntCoordLabel.setFont(font)
        self.CurruntCoordLabel.setObjectName("CurruntCoordLabel")
        self.pushButton = QtWidgets.QPushButton(Form)
        self.pushButton.setGeometry(QtCore.QRect(20, 340, 89, 25))
        self.pushButton.setCheckable(False)
        self.pushButton.setAutoDefault(False)
        self.pushButton.setDefault(False)
        self.pushButton.setFlat(False)
        self.pushButton.setObjectName("pushButton")
        self.pushButton.clicked.connect(lambda: self.SendGoalPoint(self.DesXspinBox.value(), self.DesYspinBox.value()))
        self.label = QtWidgets.QLabel(Form)
        self.label.setGeometry(QtCore.QRect(290, 166, 161, 17))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setUnderline(True)
        font.setWeight(75)
        font.setStrikeOut(False)
        self.label.setFont(font)
        self.label.setObjectName("label")
        self.DesThValue = QtWidgets.QLabel(Form)
        self.DesThValue.setGeometry(QtCore.QRect(70, 263, 67, 17))
        self.DesThValue.setObjectName("DesThValue")
        self.CurrXValue = QtWidgets.QLabel(Form)
        self.CurrXValue.setGeometry(QtCore.QRect(60, 463, 67, 17))
        self.CurrXValue.setObjectName("CurrXValue")
        self.CurrYValue = QtWidgets.QLabel(Form)
        self.CurrYValue.setGeometry(QtCore.QRect(60, 496, 67, 17))
        self.CurrYValue.setObjectName("CurrYValue")
        self.CurrThValue = QtWidgets.QLabel(Form)
        self.CurrThValue.setGeometry(QtCore.QRect(60, 527, 67, 17))
        self.CurrThValue.setObjectName("CurrThValue")

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)


    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.HorizontalLineBig.setText(_translate("Form", "BRAINOF"))
        self.DesY.setText(_translate("Form", "Y     : "))
        self.CurrX.setText(_translate("Form", "X   : "))
        self.DestinationCoordLabel.setText(_translate("Form", "Destination coordinates :"))
        self.DesX.setText(_translate("Form", "X     : "))
        self.CurrY.setText(_translate("Form", "Y   : "))
        self.CurrTh.setText(_translate("Form", "Th :"))
        self.AngleLabel.setText(_translate("Form", "*Goal angle is set as angle between current goal and previous goal."))
        self.DesTh.setText(_translate("Form", "Th* :"))
        self.CurruntCoordLabel.setText(_translate("Form", "Current coordinates :"))
        self.pushButton.setText(_translate("Form", "Set Goal"))
        self.label.setText(_translate("Form", "Generated Path"))
        self.DesThValue.setText(_translate("Form", ""))
        self.CurrXValue.setText(_translate("Form", "000"))
        self.CurrYValue.setText(_translate("Form", "000"))
        self.CurrThValue.setText(_translate("Form", "000"))
        pathstr = mapTextBoxMaker()
        self.PathPreview.setText(_translate("Form", pathstr)) 
    
    def updateCurrLocation(self, msg):
        idx = msg.name.index("robot")
        currX = msg.pose[idx].position.x
        currY = msg.pose[idx].position.y
        z = ToEulerAngles(msg.pose[idx].orientation)
        self.CurrThValue.setText(str(int(float(z)*180/math.pi)))
        self.CurrXValue.setText(str(int(currX*100/15)))
        self.CurrYValue.setText(str(int(currY*100/15)))
    
    def display_path(self, msg):
        if msg.path_len:
            pathstr = pathTextBoxStringMaker(msg.points)
            self.PathPreview.setText(pathstr) 
        else:
            self.PathPreview.setText("PATH NOT AVAILABLE ! \n") 
    
    def __init__(self): 
        rospy.init_node("GUI")
        self.sub = rospy.Subscriber("/inof/generated_path", PathPoints, self.display_path)
        self.pub = rospy.Publisher('/inof/gui_goal_points', Pose, queue_size=1)
        app = QtWidgets.QApplication(sys.argv)
        MainWindow = QtWidgets.QMainWindow()
        self.setupUi(MainWindow)
        MainWindow.show()
        sys.exit(app.exec_())   #For safe exit, close the GUI
        self.sub.unregister()   
        rospy.spin()            #And then shutdown ros. 



if __name__ == "__main__":

    ui = Ui_Form()

