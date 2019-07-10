#! /usr/bin/env python

import numpy as np
import rospy
import time

from std_msgs.msg import Float64MultiArray, Int16
from geometry_msgs.msg import Pose2D
from array import *
import matplotlib.pyplot as plt
from APF import apf

PKG = 'apf'
import roslib
roslib.load_manifest(PKG)
OBSTACLE_1 = OBSTACLE_2 = OBSTACLE_3 = OBSTACLE_4 = Pose2D()
flag_Obstacle_1 = flag_Obstacle_2 = flag_Obstacle_3 = flag_Obstacle_4 = 0



# def callback_obs_1(data):
#     global OBSTACLE_1, flag_Obstacle_1

#     if OBSTACLE_1 != data:
#         OBSTACLE_1 = data
#         flag_Obstacle_1 = 1


# def callback_obs_2(data):
#     global OBSTACLE_2, flag_Obstacle_2

#     if OBSTACLE_2 != data:
#         OBSTACLE_2 = data
#         flag_Obstacle_2 = 1



# def callback_obs_3(data):
#     global OBSTACLE_3, flag_Obstacle_3

#     if OBSTACLE_3 != data:
#         OBSTACLE_3 = data
#         flag_Obstacle_3 = 1


# def callback_obs_4(data):
#     global OBSTACLE_4, flag_Obstacle_4

#     if OBSTACLE_4 != data:
#         OBSTACLE_4 = data
#         flag_Obstacle_4 = 1

def main():
    rospy.init_node('purpleglobal_planner_node', anonymous=False)
#     rospy.Subscriber("/obst_1", Pose2D, callback_obs_1)
#     rospy.Subscriber("/obst_2", Pose2D, callback_obs_2)
#     rospy.Subscriber("/obst_3", Pose2D, callback_obs_3)
#     rospy.Subscriber("/obst_4", Pose2D, callback_obs_4)
    rate = rospy.Rate(10)
    
    bluebot = apf("/blue")
    redbot = apf("/red")
    yellowbot = apf("/yellow")
    purplebot = apf("/purple","/formation_PURPLE","/X_PURPLE","/Y_PURPLE","/THETA_PURPLE","/replan_PURPLE")
    time.sleep(1)

    while not rospy.is_shutdown():   
        if purplebot.getReplanFlagVar() >= 1:
            purplebot.plan(redbot.getPose(),yellowbot.getPose(),bluebot.getPose())
            # purplebot.setReplanFlagVar(0)
            purplebot.updatePath()
            print "purple Path is out!!!"
            rate.sleep()
            time.sleep(1)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print "ERROR !!!"