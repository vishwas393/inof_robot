#!/usr/bin/env python

import math
import sys
sys.path.append('../lib/')
from odometry import get_distance_n_angle 
from inof_robot.msg import *
from inof_robot.srv import *
import rospy
from std_msgs.msg import Float32

radius = 2
RPM = 300
dia = 2*radius

def go_to_goal(G, C):
    coord = get_distance_n_angle(G,C)
    print("Distance: " + str(coord[0]))
    print("Angle: " + str(coord[1]) + "\n")
    
    #rotate_by_degree(coord[1], wheels, False)
    

def main_application():
    rospy.init_node('open_space_mover')
    rospy.Subscriber("distance_val", Float32)
    rospy.wait_for_service("path_planning_srvc")
    try:
        client = rospy.ServiceProxy("path_planning_srvc", path_points)        
        p1 = Pose()
        p2 = Pose()
        while not rospy.is_shutdown():
            p1.x = int(input("Enter current location --> x: "))
            p1.y = int(input("Enter current location --> y: "))
            p1.t = 0
            p2.x = int(input("Enter destination location --> x: "))
            p2.y = int(input("Enter destination location --> y: "))
            p2.t = 0
            resp = client(p1, p2)
            if(resp.path_len):
                for x in resp.points:
                    print(str(x.x) + "," + str(x.y) + "\n")
            else:
                print("Path not available! \n")

    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)

    


if __name__ == '__main__':
    main_application()
    #go_to_goal([7,-4, -135],[4,0,90])
    #go_to_goal([1,-4,-135],[4,0,90])
    
