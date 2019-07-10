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

obstacleTemp = []

# def callback_obs_1(data):
#     global OBSTACLE_1, flag_Obstacle_1

#     if OBSTACLE_1 != data:
#         OBSTACLE_1 = data
#         flag_Obstacle_1 = 1
#     else:
#         flag_Obstacle_1 = 0


# def callback_obs_2(data):
#     global OBSTACLE_2, flag_Obstacle_2

#     if OBSTACLE_2 != data:
#         OBSTACLE_2 = data
#         flag_Obstacle_2 = 1
#     else:
#         flag_Obstacle_2 = 0




# def callback_obs_3(data):
#     global OBSTACLE_3, flag_Obstacle_3

#     if OBSTACLE_3 != data:
#         OBSTACLE_3 = data
#         flag_Obstacle_3 = 1
#     else:
#         flag_Obstacle_3= 0


# def callback_obs_4(data):
#     global OBSTACLE_4, flag_Obstacle_4

#     if OBSTACLE_4 != data:
#         OBSTACLE_4 = data
#         flag_Obstacle_4 = 1
#     else:
#         flag_Obstacle_4 = 0

def main():
    global flag_Obstacle_1,flag_Obstacle_2,flag_Obstacle_3,flag_Obstacle_4
    global OBSTACLE_1,OBSTACLE_2,OBSTACLE_3,OBSTACLE_4
    global obstacleTemp
    rospy.init_node('redglobal_planner_node', anonymous=False)
#     rospy.Subscriber("/obst_1", Pose2D, callback_obs_1)
#     rospy.Subscriber("/obst_2", Pose2D, callback_obs_2)
#     rospy.Subscriber("/obst_3", Pose2D, callback_obs_3)
#     rospy.Subscriber("/obst_4", Pose2D, callback_obs_4)
    rate = rospy.Rate(10)
    
    bluebot = apf("/blue")
    redbot = apf("/red","/formation_RED","/X_RED","/Y_RED","/THETA_RED","/replan_RED")
    yellowbot = apf("/yellow")
    purplebot = apf("/purple")
    time.sleep(1)

    while not rospy.is_shutdown():   
        if redbot.getReplanFlagVar() >= 1:
            # if flag_Obstacle_1 ==1:
            #         obstacleTemp.append(1)
            # else:
            #         obstacleTemp.append(0)
            # if flag_Obstacle_2 ==1:
            #         obstacleTemp.append(1)
            # else:
            #         obstacleTemp.append(0)

            # if flag_Obstacle_3 ==1:
            #         obstacleTemp.append(1)
            # else:
            #         obstacleTemp.append(0)
            # if flag_Obstacle_4 ==1:
            #         obstacleTemp.append(1)
            # else:
            #         obstacleTemp.append(0)
            # # for i in range (len(obstacleTemp)):
            # #         print "ob",i,obstacleTemp[i]
            # redbot.setObstFlag(obstacleTemp)
            redbot.plan(bluebot.getPose(),yellowbot.getPose(),purplebot.getPose())
            # redbot.setReplanFlagVar(0)
            redbot.updatePath()
            rate.sleep()
            print "red Path is out!!!"
            # print "obstacle array \t \n",redbot.getObstacles()
            # obstacleTemp = []
            # redbot.setObstFlag([0,0,0,0])

            rate.sleep()
            time.sleep(1)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print "ERROR !!!"