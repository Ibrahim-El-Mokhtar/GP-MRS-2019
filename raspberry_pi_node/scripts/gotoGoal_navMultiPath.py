#! /usr/bin/env python

import rospy
import numpy as np
import time
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64MultiArray

import smbus


# Global Variables Definitions:
global x_feedback, y_feedback, theta_feedback

global tolerance
global distance_tolerance, angle_tolerance

global flag_X, flag_Y, flag_THETA

global L, radius
global x_velocity, y_velocity, yawr_velocity

global MAX, MIN
global STEP_SIZE_TIME, CAMERA_TIME

path_X = Float64MultiArray()
path_Y = Float64MultiArray()
path_THETA = Float64MultiArray()

last_data_camera = Pose2D()


bus = smbus.SMBus(1)
time.sleep(1)
address = 0x3c


# Global Variables Defaults:
x_feedback = y_feedback = theta_feedback = 0.0

tolerance = 0.1
distance_tolerance = 0.5
#angle_tolerance = np.pi / 180
#angle_tolerance = 5.73 * (np.pi / 180)
angle_tolerance = 10

flag_X = flag_Y = flag_THETA = 0

L = 0.17					# m
radius = 0.065 / 2			# m

x_velocity = 2.0
y_velocity = 0.0
yawr_velocity = 0.0

MAX = 75
MIN = 65
STEP_SIZE_TIME = 0.055
CAMERA_TIME = 0.200


#################################################
#################################################
#################################################
#################################################
#################################################
#################################################
#################################################
#################################################
#################################################
# i2c:

def readNumber():
    global address
    number = bus.read_i2c_block_data(address,0,2)
    return number


def writeNumber(value_1, value_2, value_3):
    global address
    bus.write_i2c_block_data(address, value_1, [value_2, value_3])
    return 0
#################################################
#################################################
#################################################
#################################################
#################################################
#################################################
#################################################
#################################################
#################################################

# Callbacks:

def callback_X(data):
    global flag_X, path_X

    if path_X != data and flag_X == 0:
	path_X.data = []
        path_X.data.extend(data.data)
	print "in X_callback ***"

        flag_X = 1
#################################################


def callback_Y(data):
    global flag_Y, path_Y

    if path_Y != data and flag_Y == 0:
	path_Y.data = []
        path_Y.data.extend(data.data)
	print "in Y_callback ***"

        flag_Y = 1
#################################################


def callback_THETA(data):
    global flag_THETA, path_THETA

    if path_THETA != data and flag_THETA == 0:
	path_THETA.data = []
        path_THETA.data.extend(data.data)
	print "in THETA_callback ***"

        flag_THETA = 1
#################################################


def callback_camera_feedback(data):
    global last_data_camera
    global x_feedback, y_feedback, theta_feedback

    if last_data_camera != data:
        x_feedback = data.x
        y_feedback = data.y
	theta_feedback = data.theta

        last_data_camera = data


#################################################
#################################################
#################################################
#################################################
#################################################
#################################################
#################################################
#################################################
#################################################
# Functions:
		
def calcRpm(x_dot, y_dot, yawr_dot):
    global L, radius

    velocity = ((x_dot**2) + (y_dot)**2)**0.5
    omega = yawr_dot

    rpm_R = round((2 * velocity + L * omega) / (2 * radius))
    rpm_L = round((2 * velocity / radius) - rpm_R)

    return rpm_R, rpm_L
#################################################


def calcDistance(goal_x, goal_y):
    global x_feedback, y_feedback

    distance = (((goal_x - x_feedback)**2) + ((goal_y - y_feedback)**2))**0.5

    return distance
#################################################

def calcTheta_by_atan2(goal_x, goal_y):
	global x_feedback, y_feedback

	diff_x = goal_x - x_feedback
	diff_y = goal_y - y_feedback

	if diff_x < distance_tolerance and diff_x > 0:
		if diff_y > distance_tolerance  :
			theta = 90
		elif diff_y < distance_tolerance:
			theta = 270
		else:
			theta = 0

	else:
		theta = np.arctan2(diff_y, diff_x) * (180/np.pi)
		if theta < 0:
			theta = theta + 360

#	print "Calc_Theta_by_atan2: ", theta
	return theta



	
def calcTheta(goal_x, goal_y):
	global x_feedback, y_feedback

#    if np.abs(goal_x - x_feedback) <= distance_tolerance and goal_y > y_feedback:
	if goal_y > y_feedback + distance_tolerance  :
		if goal_x > x_feedback + distance_tolerance:
			theta = 45
	   	elif goal_x < x_feedback - distance_tolerance:
			theta = 135
	    	else:
			theta = 90

#    elif np.abs(goal_x - x_feedback) <= distance_tolerance and goal_y < y_feedback :
	elif goal_y < y_feedback-distance_tolerance :
		if goal_x > x_feedback + distance_tolerance:
			theta = 315
		elif goal_x < x_feedback - distance_tolerance:
			theta = 225
		else:
			theta = 270

#    elif np.abs(goal_y - y_feedback) <=distance_tolerance and goal_x < x_feedback:
	elif goal_x < x_feedback - distance_tolerance:
		if goal_y > y_feedback + distance_tolerance:
			theta = 135
		elif goal_y < y_feedback - distance_tolerance:
			theta = 225
	    	else:
			theta = 180
#    elif np.abs(goal_y -y_feedback) <= distance_tolerance and np.abs(goal_x - x_feedback) >= 0:
	else:
#	elif goal_x > x_feedback + distance_tolerance:
            	if goal_y > y_feedback + distance_tolerance:
                	theta = 45
            	elif goal_y < y_feedback - distance_tolerance:
                	theta = 315
	    	else:
			theta = 0
#	else:
#		theta = np.arctan2(goal_y - y_feedback, goal_x - x_feedback)
#		theta = theta * (180/np.pi)

	print "Calc_Theta: ", theta

	return theta
#################################################


def calcLinearVelocity(goal_x, goal_y, constant=1.0):
    linear_velocity = constant * calcDistance(goal_x, goal_y)

    return linear_velocity
#################################################


def calcSteeringAngle(goal_x, goal_y):
    global x_feedback, y_feedback

    diff_x = goal_x - x_feedback
    diff_y = goal_y - y_feedback

    if diff_x <= distance_tolerance:
	if diff_y > 0:
	    return 90 * (np.pi/180)
	else:
	    return 270 * (np.pi/180)

    else:
	return np.arctan2(goal_y - y_feedback, goal_x - x_feedback)

#################################################


def calcAngularVelocity(goal_x, goal_y, constant=1.0):
    global theta_feedback

    angular_velocity = constant * \
        (calcTheta_by_atan2(goal_x, goal_y) - (theta_feedback * 180))

    return angular_velocity
#################################################
#################################################
#################################################


def rpmMapping(rpm_R, rpm_L):
    global MAX, MIN

    if rpm_R > 0:
	if rpm_R < MIN:
	    rpm_R = MIN
	elif rpm_R > MAX:
	    rpm_R = MAX

    else:
	if rpm_R > -1 * MIN:
	    rpm_R = -1 * MIN
	elif rpm_R < -1 * MAX:
	    rpm_R = -1 * MAX


    if rpm_L > 0:
	if rpm_L < MIN:
	    rpm_L = MIN
	elif rpm_L > MAX:
	    rpm_L = MAX

    else:
	if rpm_L > -1 * MIN:
	    rpm_L = -1 * MIN
	elif rpm_L < -1 * MAX:
	    rpm_L = -1 * MAX

    rpm_R = np.abs(rpm_R)
    rpm_L = np.abs(rpm_L)

    return rpm_R, rpm_L
###############################################
###############################################
###############################################


def sendToTiva(rpm_R, rpm_L):
    global STEP_SIZE_TIME, CAMERA_TIME

    # 0 1
    if rpm_R > 0 and rpm_L < 0:
        command = 1

# 1 0
    elif rpm_R < 0 and rpm_L > 0:
        command = 2

# 1 1
    elif rpm_R > 0 and rpm_L > 0:
        command = 4

# 0 0
    else:
        command = 3

    rpm_R, rpm_L = rpmMapping(rpm_R, rpm_L)

#    print "\t rpm_R: ", rpm_R, "\t rpm_L: ", rpm_L, "\t command: ", command
    writeNumber(np.int(rpm_R), np.int(rpm_L), command)
    time.sleep(STEP_SIZE_TIME)

    writeNumber(0, 0, 3)
    time.sleep(CAMERA_TIME)
#################################################
#################################################
#################################################


def gotoGoal(goal_x, goal_y, goal_theta):
    global distance_tolerance, angle_tolerance
    global x_feedback, y_feedback, theta_feedback
    global x_velocity, y_velocity, yawr_velocity
    global tolerance

#    goalAngle = calcTheta(goal_x, goal_y)
 #   goalAngle = goal_theta

    dummy_counter = 0
    while dummy_counter < 1 and not rospy.is_shutdown():
#	goalAngle = goal_theta
	while (np.abs(goal_x - x_feedback) > distance_tolerance or np.abs(goal_y - y_feedback) > distance_tolerance) and not rospy.is_shutdown():
	    goalAngle = calcTheta_by_atan2(goal_x, goal_y)
#	    goalAngle = calcTheta(goal_x, goal_y)
	    while theta_feedback > goalAngle + angle_tolerance and not rospy.is_shutdown():
		goalAngle = calcTheta_by_atan2(goal_x, goal_y)
#		goalAngle = calcTheta(goal_x, goal_y)
#		print "Feedback theta: ", theta_feedback, "goal angle: ",goalAngle ,"\n"
#		print "in the FIRST ORIENTATION while() loop ..."
		yawr_velocity = calcAngularVelocity(goal_x, goal_y)
		yawr_velocity = -1 * yawr_velocity
		x_velocity = y_velocity = 0.0
		rpm_R, rpm_L = calcRpm(x_velocity, y_velocity, yawr_velocity)
		try:
		    if (goalAngle - theta_feedback >= -180) and (goalAngle - theta_feedback <= 180):
			sendToTiva(rpm_R, rpm_L)
		    else:
			sendToTiva(rpm_L, rpm_R)
		except:
                    print "BUS has fallen ... can NOT write **************"
                    time.sleep(0.300)

	    goalAngle = calcTheta_by_atan2(goal_x, goal_y)
#	    goalAngle = calcTheta(goal_x, goal_y)
	    while theta_feedback + angle_tolerance < goalAngle and not rospy.is_shutdown():
		goalAngle = calcTheta_by_atan2(goal_x, goal_y)
#		goalAngle = calcTheta(goal_x, goal_y)
#		print "in the SECOND ORIENTATION while() loop ..."
		yawr_velocity = calcAngularVelocity(goal_x, goal_y)
		x_velocity = y_velocity = 0.0
		rpm_R, rpm_L = calcRpm(x_velocity, y_velocity, yawr_velocity)
		try:
		    if (goalAngle - theta_feedback >= -180) and (goalAngle - theta_feedback <= 180):
#		    if theta_feedback <= 180:
			sendToTiva(rpm_R, rpm_L)
		    else:
			sendToTiva(rpm_L, rpm_R)
		except:
                    print "BUS has fallen ... can NOT write **************"
                    time.sleep(0.300)

#	    print "in the THIRD DISTANCE while() loop ..."
            x_velocity = calcLinearVelocity(goal_x, goal_y)
            y_velocity = yawr_velocity = 0.0
            rpm_R, rpm_L = calcRpm(x_velocity, y_velocity, yawr_velocity)
            try:
                sendToTiva(rpm_R, rpm_L)
            except:
                print "BUS has fallen ... can NOT write **************"
                time.sleep(0.300)
	
        goalAngle = goal_theta
#	goalAngle = calcTheta_by_atan2(goal_x, goal_y)
#	goalAngle = calcTheta(goal_x, goal_y)
	while theta_feedback > goalAngle + angle_tolerance and not rospy.is_shutdown():
#	    goalAngle = calcTheta_by_atan2(goal_x, goal_y)
#	    goalAngle = calcTheta(goal_x, goal_y)
#	    print "in the EXTERNAL FIRST while() loop ..."
	    yawr_velocity = calcAngularVelocity(goal_x, goal_y)
	    yawr_velocity = -1 * yawr_velocity
	    x_velocity = y_velocity = 0.0
	    rpm_R, rpm_L = calcRpm(x_velocity, y_velocity, yawr_velocity)
	    try:
		if (goalAngle - theta_feedback >= -180) and (goalAngle - theta_feedback <= 180):
		    sendToTiva(rpm_R, rpm_L)
		else:
		    sendToTiva(rpm_L, rpm_R)
	    except:
		print "BUS has fallen ... can NOT write **********************"
		time.sleep(0.300)

	goalAngle = goal_theta
#	goalAngle = calcTheta_by_atan2(goal_x, goal_y)
#	goalAngle = calcTheta(goal_x, goal_y)
	while theta_feedback + angle_tolerance < goalAngle and not rospy.is_shutdown():
#	    goalAngle = calcTheta_by_atan2(goal_x, goal_y)
#	    goalAngle = calcTheta(goal_x, goal_y)

#	    print "in the EXTERNAL SECOND while() loop ..."
	    yawr_velocity = calcAngularVelocity(goal_x, goal_y)
	    x_velocity = y_velocity = 0.0
	    rpm_R, rpm_L = calcRpm(x_velocity, y_velocity, yawr_velocity)
	    try:
		if (goalAngle - theta_feedback >= -180) and (goalAngle - theta_feedback <= 180):
		    sendToTiva(rpm_R, rpm_L)
		else:
		    sendToTiva(rpm_L, rpm_R)
	    except:
		print "BUS has fallen ... can NOT write ***************"
		time.sleep(0.300)
	
        dummy_counter = dummy_counter + 1

    print "current pose \t: ", "\t( ", x_feedback, " , ", y_feedback, " , ", theta_feedback, " )\n"
    print "GOAL IS REACHED ... FINALLY !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! \n"
    x_velocity = 0.0
    y_velocity = 0.0
    yawr_velocity = 0.0
    rpm_R, rpm_L = calcRpm(x_velocity, y_velocity, yawr_velocity)
    try:
        writeNumber(0, 0, 3)
	time.sleep(0.1)
    except:
        print "BUS has fallen ... can NOT write **************"
        time.sleep(0.300)

#################################################
#################################################
#################################################
#################################################
#################################################
#################################################
#################################################
#################################################
#################################################


def main():
    global x_feedback, y_feedback, theta_feedback
    global flag_X, flag_Y, flag_THETA
    global path_X, path_Y, path_THETA

    rospy.init_node('yellow_node', anonymous=False)
    rate = rospy.Rate(10)

    rospy.Subscriber('X_YELLOW', Float64MultiArray, callback_X)
    rospy.Subscriber('Y_YELLOW', Float64MultiArray, callback_Y)
    rospy.Subscriber('THETA_YELLOW', Float64MultiArray, callback_THETA)
    rospy.Subscriber('yellow', Pose2D, callback_camera_feedback)

    rate.sleep()

#    print "waiting 5.0 seconds before starting ....."
#    time.sleep(5.0)  # delay 5.0 seconds ...

    i = 0
    last_path_X = Float64MultiArray()
    while not rospy.is_shutdown():

        if flag_X == 1 or flag_Y == 1 or flag_THETA == 1:
            flag_X = flag_Y = flag_THETA = 0
#	    if flag_X ==1:
#	    	last_path_X = path_X
#		flag_X = 0
#           if flag_Y ==1:
#                last_path_Y = path_Y
#                flag_Y = 0
#           if flag_THETA ==1:
#                last_path_THETA = path_THETA
#                flag_THETA = 0
#	    flag_X = flag_Y = flag_THETA = 0
            print "Start from ... "
            print "\t( ", x_feedback, " , ", y_feedback, " , ", theta_feedback, " ) \n"

            print "Path to be followed ... "
            print "number of points: ", len(path_X.data)

            print "\t", "X \t: ", path_X.data, "\n"
            print "\t", "Y \t: ", path_Y.data, "\n"
            print "\t", "THETA \t: ", path_THETA.data, "\n\n"

            for i in range(len(path_X.data)):
		time.sleep(1.0)
#		if path_X != last_path_X:
		if flag_X == 1 or flag_Y == 1 or flag_THETA == 1:
		    i = 0
#		    path_X.data = path_Y.data = path_THETA.data = []
		    break
                x_goal = path_X.data[i]
                y_goal = path_Y.data[i]
                theta_goal = path_THETA.data[i]

                print "goal pose \t: ", "\t( ", x_goal, " , ", y_goal, " , ", theta_goal, " )"

                gotoGoal(x_goal, y_goal, theta_goal)

                if i == len(path_X.data) - 1:
                    flag_X = flag_Y = flag_THETA = 0
		    writeNumber(0, 0, 3)
		    time.sleep(0.020)

#	    flag_X = flag_Y = flag_THETA = 0
	    i = 0
#	    path_X.data = path_Y.data = path_THETA.data = []
	    print "ready to get a NEW path *********************************\n"

	else:
	    try:
		print "waiting for path ***"
		writeNumber(0, 0, 3)
		time.sleep(0.100)
	    except:
		print "bus has fallen in MAIN ******************* \n"
		time.sleep(0.300)

#################################################
#################################################
#################################################
#################################################
#################################################
#################################################
#################################################
#################################################
#################################################


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print "ERROR !!!"
