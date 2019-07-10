#!/usr/bin/env python

import numpy as np
import rospy
import time

from std_msgs.msg import Float64MultiArray, Int16
from geometry_msgs.msg import Pose2D
from array import *
#import matplotlib.pyplot as plt


#PKG = 'apf'
#import roslib
#roslib.load_manifest(PKG)

RED_Robot = PURPLE_Robot = BLUE_Robot = YELLOW_Robot = Pose2D()
BLUE_GOAL = PURPLE_GOAL = RED_GOAL = YELLOW_GOAL = Pose2D()
OBSTACLE_1 = OBSTACLE_2 = OBSTACLE_3 = OBSTACLE_4 = Pose2D()

KP = 8 # attractive potential gain # 8
ETA = 450.0  # repulsive potential gain #450
show_animation = True
# AREA_WIDTH = 30.0  # potential area width [m]

grid_size = 1.0  # potential grid size [m]
robot_radius = 5  # robot radius [m]

flag_robotRED = flag_robotBLUE = flag_robotYELLOW = flag_robotPURPLE = 0
flag_goalBLUE = flag_goalRED = flag_goalYELLOW = flag_goalPURPLE = 0
flag_Obstacle_1 = flag_Obstacle_2 = flag_Obstacle_3 = flag_Obstacle_4 = 0

safeZone_raduis = 4.8
tolerance = 2.5
distanceTolerance = 1
xWidth = 24
yWidth = 18


def callback_obs_1(data):
    global OBSTACLE_1, flag_Obstacle_1

    if OBSTACLE_1 != data:
        OBSTACLE_1 = data
        flag_Obstacle_1 = 1


def callback_obs_2(data):
    global OBSTACLE_2, flag_Obstacle_2

    if OBSTACLE_2 != data:
        OBSTACLE_2 = data
        flag_Obstacle_2 = 1


def callback_obs_3(data):
    global OBSTACLE_3, flag_Obstacle_3

    if OBSTACLE_3 != data:
        OBSTACLE_3 = data
        flag_Obstacle_3 = 1


def callback_obs_4(data):
    global OBSTACLE_4, flag_Obstacle_4

    if OBSTACLE_4 != data:
        OBSTACLE_4 = data
        flag_Obstacle_4 = 1



rospy.init_node('yellowLocalPlanner_node', anonymous=False)
rospy.Subscriber("/obst_1", Pose2D, callback_obs_1)
rospy.Subscriber("/obst_2", Pose2D, callback_obs_2)
rospy.Subscriber("/obst_3", Pose2D, callback_obs_3)
rospy.Subscriber("/obst_4", Pose2D, callback_obs_4)
rate = rospy.Rate(10)

class apf:
    def __init__(self ,feedbackTopic,goalTopic,replanTopic):
	
	self.goal = Pose2D()
        self.botPose=Pose2D()
        # self.obx = Float64MultiArray()
        # self.oby = Float64MultiArray()
        self.botName = feedbackTopic
        self.replanFlagVar = 0
        # rospy.init_node('global_planner_node', anonymous=False)
        rospy.Subscriber(goalTopic, Pose2D, self.GOAL)
        time.sleep(0.5)
        # rospy.Subscriber("/red", Pose2D, Robot_RED)
        # rospy.Subscriber("/formation_RED", Pose2D, GOAL_RED)


        rospy.Subscriber(feedbackTopic, Pose2D, self.currentPose)
        self.replanFlag = rospy.Publisher(replanTopic, Int16, queue_size=10)

        # rate.sleep()

    def GOAL(self,data):
        if data.x != 0 and data.y != 0:
            self.goal = data

    def currentPose(self,data):
        self.botPose = data
        # print self.botName , self.botPose 

    def getPose(self):
        return self.botPose
    
    
    """def setReplanFlagVar(self,data):
        global rate, distanceTolerance
        self.replanFlagVar = data
        if self.replanFlagVar ==1:
            if np.hypot(self.botPose.x - self.goal.x , self.botPose.y -self.goal.y )> (distanceTolerance):
                #self.replanFlagVar = data
                # if self.replanFlagVar ==1:
                self.replanFlag.publish(self.replanFlagVar)
                rate.sleep()
            else:
                self.replanFlagVar = 0
                self.replanFlag.publish(self.replanFlagVar)
                rate.sleep()
        else:
                self.replanFlag.publish(self.replanFlagVar)
                rate.sleep()
    """
    def setReplanFlagVar(self,data):
        global rate
        self.replanFlagVar = data
        # if self.replanFlagVar ==1:
        self.replanFlag.publish(self.replanFlagVar)
        rate.sleep()

    def checkReplan(self,bot1=0,bot2=0,bot3=0):
        global OBSTACLE_1, OBSTACLE_2, OBSTACLE_3, OBSTACLE_4
        global flag_Obstacle_1, flag_Obstacle_2, flag_Obstacle_3, flag_Obstacle_4
        global safeZone_raduis

        arr_obsX = Float64MultiArray()
        arr_obsY = Float64MultiArray()
        arr_obsX.data = []
        arr_obsY.data = []
        number_of_obstacles = 0
        obstacleDistArr = []

        if flag_Obstacle_1 == 1:
            # arr_obsX.data.append(OBSTACLE_1.x)
            # arr_obsY.data.append(OBSTACLE_1.y)
            number_of_obstacles = number_of_obstacles + 1
            obstacleDistArr.append(np.hypot(self.botPose.x - OBSTACLE_1.x , self.botPose.y - OBSTACLE_1.y))

        if flag_Obstacle_2 == 1:
            # arr_obsX.data.append(OBSTACLE_2.x)
            # arr_obsY.data.append(OBSTACLE_2.y)
            number_of_obstacles = number_of_obstacles + 1
            obstacleDistArr.append(np.hypot(self.botPose.x - OBSTACLE_2.x , self.botPose.y - OBSTACLE_2.y))


        if flag_Obstacle_3 == 1:
            # arr_obsX.data.append(OBSTACLE_3.x)
            # arr_obsY.data.append(OBSTACLE_3.y)
            number_of_obstacles = number_of_obstacles + 1
            obstacleDistArr.append(np.hypot(self.botPose.x - OBSTACLE_3.x , self.botPose.y - OBSTACLE_3.y))

        if flag_Obstacle_4 == 1:
            # arr_obsX.data.append(OBSTACLE_4.x)
            # arr_obsY.data.append(OBSTACLE_4.y)
            number_of_obstacles = number_of_obstacles + 1
            obstacleDistArr.append(np.hypot(self.botPose.x - OBSTACLE_4.x , self.botPose.y - OBSTACLE_4.y))

        if bot1.x!=0 and bot1.y!=0:
            # arr_obsX.data.append(bot1.x)
            # arr_obsY.data.append(bot1.y)
            number_of_obstacles = number_of_obstacles + 1
            obstacleDistArr.append(np.hypot(self.botPose.x - bot1.x , self.botPose.y - bot1.y))

        if bot2.x !=0 and bot2.y !=0:
            # arr_obsX.data.append(bot2.x)
            # arr_obsY.data.append(bot2.y)
            number_of_obstacles = number_of_obstacles + 1
            obstacleDistArr.append(np.hypot(self.botPose.x - bot2.x , self.botPose.y - bot2.y))

        if bot3.x!=0 and bot3.y!=0:
            # arr_obsX.data.append(bot3.x)
            # arr_obsY.data.append(bot3.y)
            number_of_obstacles = number_of_obstacles + 1
            obstacleDistArr.append(np.hypot(self.botPose.x - bot3.x , self.botPose.y - bot3.y))
        
        for i in range(number_of_obstacles):
            if obstacleDistArr[i] < safeZone_raduis:
                self.setReplanFlagVar(1)
                break
            else:
                self.setReplanFlagVar(0)

                
        # arr_obsX, arr_obsY = self.amplify_obstaclesArray(arr_obsX, arr_obsY, number_of_obstacles)
        # return arr_obsX, arr_obsY
def main():
    global rate
    bluebot = apf("/blue","/formation_BLUE","/replan_BLUE")
    redbot = apf("/red","/formation_RED","/replan_RED")
    yellowbot = apf("/yellow","/formation_YELLOW","/replan_YELLOW")
    purplebot = apf("/purple","/formation_PURPLE","/replan_PURPLE")
    time.sleep(1)
    
   
    while not rospy.is_shutdown():
        # bluebot.checkReplan(redbot.getPose(),yellowbot.getPose(),purplebot.getPose())
        # time.sleep(1)
        # redbot.checkReplan(bluebot.getPose(),yellowbot.getPose(),purplebot.getPose())
        # time.sleep(1)
        yellowbot.checkReplan(redbot.getPose(),bluebot.getPose(),purplebot.getPose())
        time.sleep(1)
        # purplebot.checkReplan(redbot.getPose(),yellowbot.getPose(),bluebot.getPose())
        # time.sleep(1)
        # continue


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print "ERROR !!!"
