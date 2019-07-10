#!/usr/bin/env python

"""

Potential Field based path planner

author: Atsushi Sakai (@Atsushi_twi)

Ref:
https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf

"""

import numpy as np
import matplotlib.pyplot as plt
# from scipy.signal import argelmin

# Parameters
KP = 8 # attractive potential gain
ETA = 450.0  # repulsive potential gain
# AREA_WIDTH = 30.0  # potential area width [m]
AREA_WIDTH_X = 0
AREA_WIDTH_Y = 0 
tolerance = 1
show_animation = True


def calc_potential_field(gx, gy, ox, oy, reso, rr):
    minx = 0 #min(ox) - AREA_WIDTH_X / 2.0
    miny = 0 #min(oy) - AREA_WIDTH_Y / 2.0
    maxx = 24 #max(ox) + AREA_WIDTH_X / 2.0
    maxy = 18 #max(oy) + AREA_WIDTH_Y / 2.0
    xw = int(round((maxx - minx) / reso))

    yw = int(round((maxy - miny) / reso))

    # calc each potential
    pmap = [[0.0 for i in range(yw)] for i in range(xw)]
    # ppmap = np.array(pmap)
    # print argelmin(ppmap)
    for ix in range(xw):
        x = ix * reso + minx

        for iy in range(yw):
            y = iy * reso + miny
            ug = calc_attractive_potential(x, y, gx, gy)
            uo = calc_repulsive_potential(x, y, ox, oy, rr)
            uf = ug + uo
            pmap[ix][iy] = uf

    return pmap, minx, miny


def calc_attractive_potential(x, y, gx, gy):
    return 0.5 * KP * np.hypot(x - gx, y - gy)


def calc_repulsive_potential(x, y, ox, oy, rr):
    # search nearest obstacle
    minid = -1
    dmin = float("inf")
    for i, _ in enumerate(ox):
        d = np.hypot(x - ox[i], y - oy[i])
        if dmin >= d:
            dmin = d
            minid = i

    # calc repulsive potential
    dq = np.hypot(x - ox[minid], y - oy[minid])

    if dq <= rr:
        if dq <= 0.1:
            dq = 0.1

        return 0.5 * ETA * (1.0 / dq - 1.0 / rr) ** 2
    else:
        return 0.0


def get_motion_model():
    # dx, dy
    motion = [[1, 0],
              [0, 1],
              [-1, 0],
              [0, -1],
              [-1, -1],
              [-1, 1],
              [1, -1],
              [1, 1]
              ]

    return motion

# def get_motion_model():
#     # dx, dy
#     motion = [[1, 0],
#               [0, 1],
#               [-1, 0],
#               [0, -1]]

#     return motion

def potential_field_planning(sx, sy, gx, gy, ox, oy, reso, rr):

    # calc potential field
    pmap, minx, miny = calc_potential_field(gx, gy, ox, oy, reso, rr)
    # print (pmap)
    # search path
    d = np.hypot(sx - gx, sy - gy)
    ix = round((sx - minx) / reso)
    iy = round((sy - miny) / reso)
    gix = round((gx - minx) / reso)
    giy = round((gy - miny) / reso)

    if show_animation:
        draw_heatmap(pmap)
        plt.plot(ix, iy, "*k")
        plt.plot(gix, giy, "*m")

    rx, ry = [sx], [sy]
    motion = get_motion_model()
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
        # print d
        distanceArray.append(d)
        # # print distanceArray
        # loopCount += 1 
        # if loopCount%3 == 0:
        #     loopCount = 0
        #     print "check local minima ",abs(distanceArray[0]) - abs(distanceArray[2]) #(abs(d_1 -d))
        #     if abs(distanceArray[0]) - abs(distanceArray[2]) <= 0.009 : #or abs(distanceArray[0]) - abs(distanceArray[2]) > -0.001 :
        #         print "trying to fix local minima"
        #         ox.append(xp)
        #         ox.append(rx[len(rx)-1])
        #         ox.append(rx[len(rx)-2])
        #         oy.append(yp)
        #         oy.append(ry[len(ry)-1])
        #         oy.append(ry[len(ry)-2])
        #         pmap, minx, miny = calc_potential_field(gx, gy, ox, oy, reso, rr)
        #         distanceArray = []
        #         print rx
        #         for j in range(3):
        #             rx.pop(len(rx)-1)
        #             ry.pop(len(ry)-1)
        #         print rx
        #         continue
        # distanceArray.append(d)
        # # print distanceArray
        loopCount += 1 
        if loopCount%3 == 0:
            loopCount = 0
            print "check local minima d = " , d
            print "\t\t",abs(distanceArray[0]) - abs(distanceArray[2]) #(abs(d_1 -d))
            if abs(distanceArray[0]) == abs(distanceArray[2]) : #or abs(distanceArray[0]) - abs(distanceArray[2]) > -0.001 :
                iterationCount += 1
                distanceArray = []
                if iterationCount%3 ==0:
                    iterationCount=0
                    print "trying to fix local minima"
                    ox.append(xp)
                    oy.append(yp)
                    print "Recently Appended obstacles"
                    for j in range(2):
                        ox.append(rx[len(rx)-j-1])
                        oy.append(ry[len(ry)-j-1])
                        print "[",rx[len(rx)-j-1] , "," ,ry[len(ry)-j-1] , "]" 
                    pmap, minx, miny = calc_potential_field(gx, gy, ox, oy, reso, rr)
                    print rx , ry
                    for k in range(9):
                        rx.pop(len(rx)-1)
                        ry.pop(len(ry)-1)
                    print rx , ry
                    continue
            else:
                distanceArray = []
        rx.append(xp)
        ry.append(yp)
        # print xp , yp
        if show_animation:
            # plt.plot()
            plt.plot(ix, iy, ".r")
            plt.pause(0.01)

    print("Goal!!")

    return rx, ry


def draw_heatmap(data):
    data = np.array(data).T
    plt.pcolor(data, vmax=100.0, cmap=plt.cm.Blues)

def amplify_obstaclesArray(arr_obsX, arr_obsY, number_of_obstacles):
        global tolerance
        temp_X = []
        temp_Y =[]
        # temp_X.data = temp_Y.data = []
        # current_x = current_y = 0.0
        i=0
        current_x = []
        current_y = []
        current_x = arr_obsX 
        current_y = arr_obsY
        arr_obsX = []
        arr_obsY = []
        
        for i in range(number_of_obstacles):
            current_x[i] #= arr_obsX[i]
            current_y[i] #= arr_obsY[i]
            """
                7	8	9
                4	5	6
                1	2	3

                6   7   8
                3   4   5
                0   1   2
                x-1,y-1	x,y  x+1,y
                x-1,y	x,y  x+1,y
                x-1,y+1	x,y  x+1,y
            """
            j=0
            
            for j in range(9):
                if j == 1-1 or j == 4-1 or j == 7-1:
                # if j < 3:
                    temp_X.append(current_x[i] - tolerance)
                # elif j >= 3 and j < 6:
                elif j == 2-1 or j == 5-1 or j == 8-1:
                    temp_X.append(current_x[i])
                else:
                    temp_X.append(current_x[i] + tolerance)
                # print temp_X , temp_Y
            j=0
            arr_obsX.extend(temp_X)
            temp_X =  []
            for j in range(9):
                # if j == 1 or j == 4 or j == 7:
                if j < 3:
                    temp_Y.append(current_y[i] - tolerance)
                # elif j == 2 or j == 5 or j == 8:
                elif j >= 3 and j < 6:
                    temp_Y.append(current_y[i])
                else:
                    temp_Y.append(current_y[i] + tolerance)
            arr_obsY.extend(temp_Y)
            temp_Y = []

        print "OBSTACLE ARRAY",arr_obsX, arr_obsY
        return arr_obsX, arr_obsY

def main():
    print("potential_field_planning start")

    sx = 7  # start x position [m]
    sy = 4  # start y positon [m]
    gx = 11 # goal x position [m]
    gy = 14 # goal y position [m]
    grid_size = 1  # potential grid size [m]
    robot_radius = 6.5  # robot radius [m]
    # ox = oy = []
    # ox , oy = amplify_obstaclesArray([1],[12],1)

    # ox , oy =amplify_obstaclesArray([1, 5.0, 5, 5.0],[12, 8.0, 10.0, 12.0],4)
    ox = [ 5, 7.5 ,10 ]  # obstacle x position list [m]
    oy = [10.0,13, 10.0]  # obstacle y position list [m]
    # ox = [6.67, 6.67, 16.17]  # obstacle x position list [m]
    # oy = [5.96, 12.44, 9.20]  # obstacle y position list [m]
    # ox = [1]
    # oy= [12]
    if show_animation:
        plt.grid(True)
        plt.axis("equal")
    
    # path generation
    pathX, pathY = potential_field_planning(
        sx, sy, gx, gy, ox, oy, grid_size, robot_radius)
    print pathX 
    print pathY
    if show_animation:
        plt.show()


if __name__ == '__main__':
    print(__file__ + " start!!")
    main()
    print(__file__ + " Done!!")

