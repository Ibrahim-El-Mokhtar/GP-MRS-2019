#! /usr/bin/env python

import numpy as np
import rospy
import time

from std_msgs.msg import Float64MultiArray, Int16
from geometry_msgs.msg import Pose2D
from array import *
import matplotlib.pyplot as plt


PKG = 'apf'
import roslib
roslib.load_manifest(PKG)

OBSTACLE_1 = OBSTACLE_2 = OBSTACLE_3 = OBSTACLE_4 = Pose2D()

KP = 9 # attractive potential gain # 8
ETA = 900  # repulsive potential gain #450
show_animation = False
# AREA_WIDTH = 30.0  # potential area width [m]

grid_size = 1.0  # potential grid size [m]
robot_radius = 6.5 #4.5+0.3  # robot radius [m]

distanceTolerance = 1


flag_Obstacle_1 = flag_Obstacle_2 = flag_Obstacle_3 = flag_Obstacle_4 = 0

safeZone_raduis = 3 #3.6 # clearence distance with last replan pose
tolerance = 2.5
xWidth = 24
yWidth = 18



class apf:
    def __init__(self , feedbackTopic,goalTopic=0,xPathTopic=0,yPathTopic=0,thetaPathTopic=0,replanTopic=0):
        
        self.goal=Pose2D()
        self.botPose=Pose2D()
        self.replanBotPose = Pose2D()
        self.xPath = Float64MultiArray()
        self.yPath = Float64MultiArray()
        self.thetaPath = Float64MultiArray()
        self.obx = Float64MultiArray()
        self.oby = Float64MultiArray()
        self.botName = feedbackTopic
        self.replanFlagVar = 0
        # self.flag_Obstacle_1=0
        # self.flag_Obstacle_2=0
        # self.flag_Obstacle_3=0
        # self.flag_Obstacle_4=0
        # rospy.init_node('redglobal_planner_node', anonymous=False)
        rospy.Subscriber("/obst_1", Pose2D, self.callback_obs_1)
        rospy.Subscriber("/obst_2", Pose2D, self.callback_obs_2)
        rospy.Subscriber("/obst_3", Pose2D, self.callback_obs_3)
        rospy.Subscriber("/obst_4", Pose2D, self.callback_obs_4)

        
        # rospy.init_node('global_planner_node', anonymous=False)
        time.sleep(0.5)
        # rospy.Subscriber("/red", Pose2D, Robot_RED)
        # rospy.Subscriber("/formation_RED", Pose2D, GOAL_RED)


        rospy.Subscriber(feedbackTopic, Pose2D, self.currentPose)
        if goalTopic !=0:
            rospy.Subscriber(goalTopic, Pose2D, self.GOAL)
        if replanTopic !=0:    
            rospy.Subscriber(replanTopic, Int16, self.replanFlag)
        # rate.sleep()
        if xPathTopic !=0:
            self.pubX = rospy.Publisher(xPathTopic, Float64MultiArray, queue_size=10)
        if yPathTopic !=0:
            self.pubY = rospy.Publisher(yPathTopic, Float64MultiArray, queue_size=10)
        if thetaPathTopic !=0:
            self.pubTHETA = rospy.Publisher(thetaPathTopic, Float64MultiArray, queue_size=10)

    def GOAL(self,data):
        if data.x != 0 and data.y != 0:
            if self.goal != data:
                self.setReplanFlagVar(2)
                self.goal = data
                
        # print "GOAL" , self.goal
    def callback_obs_1(self,data):
        global OBSTACLE_1
        global flag_Obstacle_1
        
        if OBSTACLE_1 != data and data.x >0:
            OBSTACLE_1 = data
            flag_Obstacle_1 = 1
            # print "OB1!!!" ,OBSTACLE_1
            # print "OB 1 FLAGG !!",self.flag_Obstacle_1
        else:
            flag_Obstacle_1 = 0

    def callback_obs_2(self,data):
        global OBSTACLE_2,flag_Obstacle_2

        if OBSTACLE_2 != data and data.x >0:
            OBSTACLE_2 = data
            flag_Obstacle_2 = 1
            # print "OB2!!!",OBSTACLE_2

        else:
            flag_Obstacle_2 = 0

    def callback_obs_3(self,data):
        global OBSTACLE_3, flag_Obstacle_3

        if OBSTACLE_3 != data and data.x >0:
            OBSTACLE_3 = data
            flag_Obstacle_3 = 1
            # print "OB3!!!",OBSTACLE_3

        else:
            flag_Obstacle_3 = 0

    def callback_obs_4(self,data):
        global OBSTACLE_4 , flag_Obstacle_4

        if OBSTACLE_4 != data and data.x >0:
            OBSTACLE_4 = data
            flag_Obstacle_4 = 1
            # print "OB4!!!",OBSTACLE_4
        else:
            flag_Obstacle_4 = 0

    def getObstacles(self):
        return self.obx.data , self.oby.data

    # def setObstFlag(self,ob=[]):
    #     self.flag_Obstacle_1 = ob[0]
    #     self.flag_Obstacle_2 = ob[1]
    #     self.flag_Obstacle_3 = ob[2]
    #     self.flag_Obstacle_4 = ob[3]  
    def getGoal(self):
        return self.goal

    def currentPose(self,data):
        self.botPose = data
        # print self.botName , self.botPose 

    def getPose(self):
        return self.botPose
    
    def replanFlag(self, data):
        if data.data == 1:
            self.replanFlagVar = 1
        # else:
        #     self.replanFlagVar = 0
    
    def setReplanFlagVar(self,data):
        self.replanFlagVar = data

    def getReplanFlagVar(self):
        return self.replanFlagVar
        
    def setReplanPose(self, data):
        self.replanBotPose = data

    def plan(self,bot1=0,bot2=0,bot3=0 , virtualObstacle=[]):
        global safeZone_raduis,distanceTolerance
        print "\t ********** ",self.botName 
        if np.hypot(self.botPose.x - self.goal.x , self.botPose.y -self.goal.y )> (distanceTolerance) :
            if np.hypot(self.replanBotPose.x-self.botPose.x , self.replanBotPose.y-self.botPose.y)> (safeZone_raduis) and self.replanFlagVar ==1:
                print "replanning !! \n"
                self.generate_Obstacles(bot1,bot2,bot3) #,virtualObstacle)
                self.getPath()
                self.setReplanFlagVar(0)
                # self.updatePath()
                self.replanBotPose = self.botPose
            elif self.replanFlagVar == 2 :
                self.generate_Obstacles(bot1,bot2,bot3) #,virtualObstacle)
                self.getPath()
                self.setReplanFlagVar(0)
                # self.updatePath()
                self.replanBotPose = self.botPose
            else:
                print "will replan after a bit ... just Replanned "
                self.setReplanFlagVar(0)

        else:
            print "WON'T replan ... GOAL REACHED !!!!!! "
            self.setReplanFlagVar(0)

    def calc_potential_field(self,gx, gy, ox, oy, reso, rr):
        global AREA_WIDTH, xWidth, yWidth

        minx = 0 #min(ox.data) - AREA_WIDTH / 2.0
        miny = 0 #min(oy.data) - AREA_WIDTH / 2.0

        maxx = xWidth #max(ox.data) + AREA_WIDTH / 2.0
        maxy = yWidth #max(oy.data) + AREA_WIDTH / 2.0

        xw = int(round((maxx - minx) / reso))
        yw = int(round((maxy - miny) / reso))

        pmap = [[0.0 for i in range(yw)] for i in range(xw)]

        for ix in range(xw):
            x = ix * reso + minx
            for iy in range(yw):
                y = iy * reso + miny
                ug = self.calc_attractive_potential(x, y, gx, gy)
                uo = self.calc_repulsive_potential(x, y, ox, oy, rr)
                uf = ug + uo
                pmap[ix][iy] = uf
        return pmap, minx, miny





    def calc_attractive_potential(self,x, y, gx, gy):
        global KP

        return 0.5 * KP * np.hypot(x - gx, y - gy)


    def calc_repulsive_potential(self,x, y, ox, oy, rr):
        global ETA

        minid = -1
        dmin = float("inf")
        for i, _ in enumerate(ox.data):
            d = np.hypot(x - ox.data[i], y - oy.data[i])
            if dmin >= d:
                dmin = d
                minid = i

        dq = np.hypot(x - ox.data[minid], y - oy.data[minid])

        if dq <= rr:
            if dq <= 0.1:
                dq = 0.1

            return 0.5 * ETA * (1.0 / dq - 1.0 / rr) ** 2
        else:
            return 0.0


    def get_motion_model(self):
        # dx, dy
        motion = [[1, 0],
                [0, 1],
                [-1, 0],
                [0, -1],
                [-1, -1],
                [-1, 1],
                [1, -1],
                [1, 1]]
        return motion
    # def get_motion_model(self):
    #     # dx, dy
    #     motion = [[1, 0],
    #             [0, 1],
    #             [-1, 0],
    #             [0, -1]]
    #     return motion

    def potential_field_planning(self,sx, sy, gx, gy, ox, oy, reso, rr):
        # calc potential field
        pmap, minx, miny = self.calc_potential_field(gx, gy, ox, oy, reso, rr)

        # search path
        d = np.hypot(sx - gx, sy - gy)
        # d = np.hypot(gx - sx, gy - sy)

        ix = round((sx - minx) / reso)
        iy = round((sy - miny) / reso)
        gix = round((gx - minx) / reso)
        giy = round((gy - miny) / reso)
        if show_animation:
            self.draw_heatmap(pmap)
            plt.plot(ix, iy, "*k")
            plt.plot(gix, giy, "*m")
        rx, ry = [sx], [sy]
        motion = self.get_motion_model()
        distanceArray = []
        loopCount = 0
        iterationCount =0
        while d >= reso:
            minp = float("inf")
            minix, miniy = -1, -1
            for i, _ in enumerate(motion):
                inx = int(ix + motion[i][0])
                iny = int(iy + motion[i][1])
                if inx >= len(pmap) or iny >= len(pmap[0]):
                    p = float("inf")  # outside area
                else:
                    p = pmap[inx][iny]
                if minp > p:
                    minp = p
                    minix = inx
                    miniy = iny
            ix = minix
            iy = miniy
            xp = ix * reso + minx
            yp = iy * reso + miny
            d = np.hypot(gx - xp, gy - yp)
            distanceArray.append(d)
            loopCount += 1 
            if loopCount%3 == 0:
                loopCount = 0
                #print "check local minima d = " , d
                print "\t\t",abs(distanceArray[0]) - abs(distanceArray[2]) #(abs(d_1 -d))
                if abs(distanceArray[0]) == abs(distanceArray[2]) : #or abs(distanceArray[0]) - abs(distanceArray[2]) > -0.001 :
                    iterationCount += 1
                    distanceArray = []
                    if iterationCount%3 ==0:
                        iterationCount=0
                        #print "trying to fix local minima"
                        ox.data.append(xp)
                        oy.data.append(yp)
                        #print "Recently Appended obstacles"
                        for j in range(2):
                            ox.data.append(rx[len(rx)-j-1])
                            oy.data.append(ry[len(ry)-j-1])
                            print "[",rx[len(rx)-j-1] , "," ,ry[len(ry)-j-1] , "]" 
                        pmap, minx, miny = self.calc_potential_field(gx, gy, ox, oy, reso, rr)
                        print "RX RY BEFORE POP",rx , ry
                        for k in range(8):
                            rx.pop(len(rx)-1)
                            ry.pop(len(ry)-1)
                        print "RX RY AFTER POP",rx , ry
                        continue
                else:
                    distanceArray = []
            rx.append(xp)
            ry.append(yp)
            if show_animation:
                plt.plot(ix, iy, ".r")
                plt.pause(0.01)
        return rx, ry


    def calcThetaArray_by_atan2(self, arr_X, arr_Y):
        calcThetaArray = Float64MultiArray()
        for i in range(len(arr_X) - 1):
            x_start = arr_X[i]
            y_start = arr_Y[i]
            x_goal = arr_X[i + 1]
            y_goal = arr_Y[i + 1]

            x_diff = x_goal - x_start
            y_diff = y_goal - y_start

            distance_tolerance = 0.5
            if x_diff < distance_tolerance and x_diff > 0:
                if y_diff > 0:
                    theta_goal = 90
                elif y_diff < 0:
                    theta_goal = 270
                else:
                    theta_goal = 0

            else:
                theta_goal = np.arctan2(y_diff, x_diff)
                theta_goal = theta_goal * (180 / np.pi)
                if theta_goal < 0:
                    theta_goal = theta_goal + 360
            calcThetaArray.data.append(theta_goal)
        calcThetaArray.data.append(self.goal.theta)

        return calcThetaArray


    def amplify_obstaclesArray(self,arr_obsX, arr_obsY, number_of_obstacles):
        global tolerance

        temp_X = temp_Y = Float64MultiArray()
        temp_X.data = []
        temp_Y.data = []
        current_x = current_y = 0.0

        for i in range(number_of_obstacles):
            current_x = arr_obsX.data[i]
            current_y = arr_obsY.data[i]

            """
                7	8	9
                4	5	6
                1	2	3
            
                x-1,y-1	x,y  x+1,y
                x-1,y	x,y  x+1,y
                x-1,y+1	x,y  x+1,y
            """

            for j in range(9):
                if j < 3:
                    temp_X.data.append(current_x - tolerance)
                elif j >= 3 and j < 6:
                    temp_X.data.append(current_x)
                else:
                    temp_X.data.append(current_x + tolerance)

            for j in range(9):
                if j == 1 or j == 4 or j == 7:
                    temp_Y.data.append(current_y - tolerance)
                elif j == 2 or j == 5 or j == 8:
                    temp_Y.data.append(current_y)
                else:
                    temp_Y.data.append(current_y + tolerance)

        arr_obsX = temp_X
        arr_obsY = temp_Y

        return arr_obsX, arr_obsY


    def generate_Obstacles(self,bot1=0,bot2=0,bot3=0,virtualObstacle=[]):
        global OBSTACLE_1, OBSTACLE_2, OBSTACLE_3, OBSTACLE_4
        global flag_Obstacle_1, flag_Obstacle_2, flag_Obstacle_3, flag_Obstacle_4

        arr_obsX = Float64MultiArray()
        arr_obsY = Float64MultiArray()
        arr_obsX.data = []
        arr_obsY.data = []
        number_of_obstacles = 0

        print "FLAGGGGG",flag_Obstacle_1,flag_Obstacle_2 , flag_Obstacle_3 , flag_Obstacle_4 
        
        if flag_Obstacle_1 == 1:
            arr_obsX.data.append(OBSTACLE_1.x)
            arr_obsY.data.append(OBSTACLE_1.y)
            number_of_obstacles = number_of_obstacles + 1

        if flag_Obstacle_2 == 1:
            arr_obsX.data.append(OBSTACLE_2.x)
            arr_obsY.data.append(OBSTACLE_2.y)
            number_of_obstacles = number_of_obstacles + 1

        if flag_Obstacle_3 == 1:
            arr_obsX.data.append(OBSTACLE_3.x)
            arr_obsY.data.append(OBSTACLE_3.y)
            number_of_obstacles = number_of_obstacles + 1

        if flag_Obstacle_4 == 1:
            arr_obsX.data.append(OBSTACLE_4.x)
            arr_obsY.data.append(OBSTACLE_4.y)
            number_of_obstacles = number_of_obstacles + 1

        if bot1.x!=0 and bot1.y!=0:
            arr_obsX.data.append(bot1.x)
            arr_obsY.data.append(bot1.y)
            number_of_obstacles = number_of_obstacles + 1

        if bot2.x !=0 and bot2.y !=0:
            arr_obsX.data.append(bot2.x)
            arr_obsY.data.append(bot2.y)
            number_of_obstacles = number_of_obstacles + 1

        if bot3.x!=0 and bot3.y!=0:
            arr_obsX.data.append(bot3.x)
            arr_obsY.data.append(bot3.y)
            number_of_obstacles = number_of_obstacles + 1
        if number_of_obstacles == 0:
            arr_obsX = [0]
            arr_obsY = [0]
            number_of_obstacles = number_of_obstacles + 1

        # if len(virtualObstacle) !=0:
        #     for i in range(len(virtualObstacle)):
        #         if virtualObstacle[i].x !=0 and virtualObstacle[i].y !=0:
        #             arr_obsX.data.append(virtualObstacle[i].x)
        #             arr_obsY.data.append(virtualObstacle[i].y)
        #             number_of_obstacles = number_of_obstacles + 1
        # arr_obsX, arr_obsY = self.amplify_obstaclesArray(arr_obsX, arr_obsY, number_of_obstacles)
        # return arr_obsX, arr_obsY
        self.obx =  arr_obsX
        self.oby =  arr_obsY
        print "obs X",self.obx.data,"obs Y", self.oby.data

    def getPath (self):
        global grid_size, robot_radius 
        print "will calculate \n"
        self.xPath.data , self.yPath.data = self.potential_field_planning (self.botPose.x,self.botPose.y,self.goal.x, self.goal.y, self.obx,self.oby, grid_size, robot_radius)
        print "Path Genrated"
        self.thetaPath = self.calcThetaArray_by_atan2(self.xPath.data,self.yPath.data)
        print "x :" , self.xPath.data , "\n y :", self.yPath.data
        # self.updatePath() 

    def updatePath(self):
        # global rate
        print "publishing"
        if show_animation:
            plt.grid(True)
            plt.axis("equal")
        self.pubX.publish(self.xPath)
        self.pubY.publish(self.yPath)
        self.pubTHETA.publish(self.thetaPath)  
        # rate.sleep()      
    
    def draw_heatmap(self,data):
        data = np.array(data).T
        plt.pcolor(data, vmax=100.0, cmap=plt.cm.Blues)


