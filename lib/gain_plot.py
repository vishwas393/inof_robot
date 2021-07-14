#!/usr/bin/env python

import rospy
import os
import time
import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose, Vector3, Twist
from gazebo_msgs.msg import ModelState, ModelStates
from inof_robot.msg import Pose

time = []
position_x = []
position_y = []

fid = 0
init_time = 0

def callback_cmd(msg):
    fid = msg.name.index("robot")

    curr = ModelState()
    curr.pose = msg.pose[fid]
    curr.twist = msg.twist[fid]

    time.append(rospy.get_time() - init_time)
    position_x.append(curr.pose.position.x)
    position_y.append(curr.pose.position.y)

    #rospy.loginfo("time array len: " + str(len(time)) + "   position array len: " + str(len(position)))


def main():
    rospy.init_node('gain_plotter')
    init_time = rospy.get_time()
    sub_cmd = rospy.Subscriber("/gazebo/model_states", ModelStates, callback_cmd)
    rospy.spin()
    plt.plot(position_x, position_y, label="X-position")
    #plt.plot(time, position_y, label="Y-position")
    plt.legend()
    plt.title("k1=25 k2=1 k3=10")
    plt.show()

if __name__ == "__main__":
    main()

