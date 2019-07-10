#!/usr/bin/env python

import rospy
import random
import time
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64MultiArray

## --------------------------------- global variables ---------------------------------
step_size = 6.0
number_of_shapes = 8

activated_robots = Float64MultiArray()
# activated = [PURPLE, RED, BLUE, YELLOW]
activated_robots.data = [-1, -1, -1, -1] 
ROBOT_NAME = ["PURPLE", "RED", "BLUE", "YELLOW"]
number_of_robots = 0
selected_shape_name = "none"
selected_shape_nameArray = ["Line", "Column", "Diagonal_45", "Diagonal_135",\
    "Triangle", "L-Shape", "Square", "Diamond", "NONE"]
selection_limits = [-1, -1]
global_dummy_counter = 0

X_detected = Float64MultiArray()
Y_detected = Float64MultiArray()
THETA_detected = Float64MultiArray()

X_detected.data = [-1, -1, -1, -1]
Y_detected.data = [-1, -1, -1, -1]
THETA_detected.data = [-1, -1, -1, -1]

known_points = Float64MultiArray()
flag_selectionArray = Float64MultiArray()
flag_selectionArray.data = [-1, -1, -1, -1]

distance_PURPLE = Float64MultiArray()
distance_RED = Float64MultiArray()
distance_BLUE = Float64MultiArray()
distance_YELLOW = Float64MultiArray()

X_f = Float64MultiArray()
Y_f = Float64MultiArray()
THETA_f = Float64MultiArray()

goal_PURPLE = Pose2D()
goal_RED = Pose2D()
goal_BLUE = Pose2D()
goal_YELLOW = Pose2D()

## ------------------------------------------------------------------------
## --------------------------------- callback -----------------------------
## ------------------------------------------------------------------------
def callback_PURPLE(data):
    global X_detected, Y_detected, THETA_detected
    global activated_robots

    robot_id = 1
    index = robot_id - 1

    time.sleep(0.100)

    if data.x <= 0 or data.y <= 0:
        activated_robots.data[index] = -1
    else:
        X_detected.data[index] = data.x
        Y_detected.data[index] = data.y
        THETA_detected.data[index] = data.theta

        activated_robots.data[index] = 1

#    print "activated_robots: ", activated_robots.data
#    time.sleep(0.100)

    #if (data.y < 14.0) and (activated_robots.data[index] != 1):
#    if (activated_robots.data[index] != 1) or (X_detected.data[index] != data.x) or (Y_detected.data[index] != data.y) or (THETA_detected.data[index] != data.theta):
#        X_detected.data[index] = data.x
#        Y_detected.data[index] = data.y
#        THETA_detected.data[index] = data.theta

#        activated_robots.data[index] = 1

#    else:
#        activated_robots.data[index] = -1


### ****************************************************************************
def callback_RED(data):
    global X_detected, Y_detected, THETA_detected
    global activated_robots

    robot_id = 2
    index = robot_id - 1

    time.sleep(0.100)

    if data.x <= 0 or data.y <= 0:
        activated_robots.data[index] = -1
    else:
        X_detected.data[index] = data.x
        Y_detected.data[index] = data.y
        THETA_detected.data[index] = data.theta

        activated_robots.data[index] = 1
    
#    print "activated_robots: ", activated_robots.data
#    time.sleep(0.100)

#    if (data.y < 14.0) and (activated_robots.data[index] != 1):
#    if (activated_robots.data[index] != 1) or (X_detected.data[index] != data.x) or (Y_detected.data[index] != data.y) or (THETA_detected.data[index] != data.theta):
#        X_detected.data[index] = data.x
#        Y_detected.data[index] = data.y
#        THETA_detected.data[index] = data.theta

#        activated_robots.data[index] = 1

#    else:
#        activated_robots.data[index] = -1


### ****************************************************************************
def callback_BLUE(data):
    global X_detected, Y_detected, THETA_detected
    global activated_robots

    robot_id = 3
    index = robot_id - 1

    time.sleep(0.100)

    if data.x <= 0 or data.y <= 0:
        activated_robots.data[index] = -1
    else:
        X_detected.data[index] = data.x
        Y_detected.data[index] = data.y
        THETA_detected.data[index] = data.theta

        activated_robots.data[index] = 1

#    print "activated_robots: ", activated_robots.data
#    time.sleep(0.100)

###    if (data.x > 1.0) and (data.y > 1.0) and (activated_robots.data[index] != 1):
#    if (activated_robots.data[index] != 1) or (X_detected.data[index] != data.x) or (Y_detected.data[index] != data.y) or (THETA_detected.data[index] != data.theta):
#        X_detected.data[index] = data.x
#        Y_detected.data[index] = data.y
#        THETA_detected.data[index] = data.theta

#        activated_robots.data[index] = 1

#    else:
#        activated_robots.data[index] = -1


### ****************************************************************************
def callback_YELLOW(data):
    global X_detected, Y_detected, THETA_detected
    global activated_robots

    robot_id = 4
    index = robot_id - 1

    time.sleep(0.100)

    if data.x <= 0 or data.y <= 0:
        activated_robots.data[index] = -1
    else:
        X_detected.data[index] = data.x
        Y_detected.data[index] = data.y
        THETA_detected.data[index] = data.theta

        activated_robots.data[index] = 1
    
#    print "activated_robots: ", activated_robots.data
#    time.sleep(0.100)

#    if (data.y < 14.0) and (activated_robots.data[index] != 1):
#    if (activated_robots.data[index] != 1) or (X_detected.data[index] != data.x) or (Y_detected.data[index] != data.y) or (THETA_detected.data[index] != data.theta):
#        X_detected.data[index] = data.x
#        Y_detected.data[index] = data.y
#        THETA_detected.data[index] = data.theta

#        activated_robots.data[index] = 1

#    else:
#        activated_robots.data[index] = -1

## ------------------------------------------------------------------------
## --------------------------------- formation functions ------------------
## ------------------------------------------------------------------------
def printing_goal_points():
    global X_f, Y_f, THETA_f

    for i in range(len(X_f.data)):
        print "P", i+1, " = ", "( ", X_f.data[i], " , ", Y_f.data[i], " , ", THETA_f.data[i], " )"


### ****************************************************************************
def find_min_shape_valid_number():
    global number_of_robots, selection_limits

    if number_of_robots <= 2:
        selection_limits = [1, 4]
    elif number_of_robots == 3:
        selection_limits = [1, 5]
    elif number_of_robots == 4:
        selection_limits = [1, 8]


### ****************************************************************************
def check_valid_shapes(shape_number):
    global number_of_robots

    if number_of_robots > 0:
        find_min_shape_valid_number()



### ****************************************************************************
def check_active_robots(shape_number):
    global activated_robots, number_of_robots
    global ROBOT_NAME, selected_shape_name

    time.sleep(0.100)

    if sum(activated_robots.data) == -4:
            number_of_robots = 0

    elif sum(activated_robots.data) == -2:
        number_of_robots = 1

    elif sum(activated_robots.data) == 0:
        number_of_robots = 2

    elif sum(activated_robots.data) == 2:
        number_of_robots = 3
    
    elif sum(activated_robots.data) == 4:
        number_of_robots = 4

    else:
        print "SOMETHING IS GOING WRONG ***************************"

    print "\n"
    if shape_number >= selection_limits[0] and shape_number <= selection_limits[1]:
        print "current chosen shape: ", selected_shape_name
        print "number of robots: ", number_of_robots
        robot_out = 0
        for i in range(len(activated_robots.data)):
            if activated_robots.data[i] == -1:
                robot_out = 1
                print "\t\t robot ", ROBOT_NAME[i], "\t is OUT ........"  
        if robot_out != 1:
            print "\t ALL THE ROBOTS ARE IN ..."   

    check_valid_shapes(shape_number)               

    activated_robots.data = [-1, -1, -1, -1]
    

## ------------------------------------------------------------------------
## --------------------------------- shapes -------------------------------
## ------------------------------------------------------------------------
def select_shape():
    global number_of_robots, number_of_shapes
    global selected_shape_name
    global selected_shape_nameArray
    global flag_selectionArray

    shape_number = input("\tchoose a shape to form: \n\
        \t1. Line \t\t 2. Column \t\t 3. Diagonal_45\n\
        \t4. Diagonal_135 \t 5. Triangle \t\t 6. L-Shape\n\
        \t7. Square \t\t 8. Diamond \t\t 0. exit \n\
        \t Selected shape: ")

    if shape_number > number_of_shapes or shape_number < 0:
        print "\n"
        print "KINDLY SELECT A VALID SHAPE ********************* \n"
        select_shape()

    else:
        if shape_number == 0 or shape_number > number_of_shapes:
            last_name = len(selected_shape_nameArray) - 1
            selected_shape_name = selected_shape_nameArray[last_name]

        else:
            selected_shape_name = selected_shape_nameArray[shape_number - 1]

    return shape_number


### ****************************************************************************
def regenrate_shape(chosen_shape, pub_PURPLE, pub_RED, pub_BLUE, pub_YELLOW):
    global number_of_robots, selection_limits, selected_shape_name
    global flag_selectionArray

    regeneration_break = 0

    while chosen_shape >= selection_limits[0] and chosen_shape <= selection_limits[1] and not rospy.is_shutdown():
        print "inside regeneration function ... regenerating a ", selected_shape_name
        time.sleep(0.150)
        check_active_robots(chosen_shape)
        draw_selected_shape(chosen_shape, pub_PURPLE, pub_RED, pub_BLUE, pub_YELLOW)
        time.sleep(0.010)   ## it's acting like:     rate.sleep()
        regeneration_break = input("if you want to break this shape_keeping press 0 ... \t answer: ")
        if regeneration_break == 0:
            break
        else:
            check_active_robots(chosen_shape)
            continue

    print "\n\n"
    print "CAN NOT GENERATE THE SAME SHAPE ANY MORE *******************"
    selected_shape_name = "NONE"
    return regeneration_break


def set_starting_conditions(shape_number):
    global line_angle
    global flag_selectionArray

    print "SETTING SOME INITIAL CONDITIONS ..."
    time.sleep(0.020)
    start_pose = Pose2D()

    if shape_number == 1:
        scaling_factor = 2.2 / 2
        start_pose.x = 4.0
        start_pose.y = 9.0
        start_pose.theta = 0.0


    elif shape_number == 2:
        scaling_factor = 1.7 / 2
        start_pose.x = 12.0
        start_pose.y = 1.6
        start_pose.theta = 90.0



    elif shape_number == 3:
        scaling_factor = 1.9 / 2
        start_pose.x = 6.0
        start_pose.y = 2.2
        start_pose.theta = 45.0


    elif shape_number == 4:
        scaling_factor = 1.9 / 2
        start_pose.x = 18.0
        start_pose.y = 2.2
        start_pose.theta = 135.0

    elif shape_number == 5:
        scaling_factor = 5.5
        start_pose.x = 14.0
        start_pose.y = 9.0
        start_pose.theta = 0.0

    elif shape_number == 6:
        scaling_factor = 2.5 / 2
        start_pose.x = 6.0
        start_pose.y = 7.0
        start_pose.theta = 0.0

    elif shape_number == 7:
        scaling_factor = 3.5 / 2
        start_pose.x = 14.0
        start_pose.y = 9.0
        start_pose.theta = 0.0

    elif shape_number == 8:
        scaling_factor = 1.8 / 2
        start_pose.x = 14.0
        start_pose.y = 9.5
        start_pose.theta = 0.0

    else:
        print "THERE IS SOMETHING WONRG *****************"

    
    return start_pose, scaling_factor


### ****************************************************************************
def draw_selected_shape(shape_number, pub_PURPLE, pub_RED, pub_BLUE, pub_YELLOW):
    global global_dummy_counter
    global line_angle
    global distance_PURPLE, distance_RED, distance_BLUE, distance_YELLOW
    global flag_selectionArray

    start_pose = Pose2D()
    scaling_factor = 0.0

    if shape_number == 0:
        return 0

    else:
        start_pose, scaling_factor = set_starting_conditions(shape_number)
        
        if shape_number == 1:
            draw_line(start_pose, scaling_factor)

        elif shape_number == 2:
            draw_column(start_pose, scaling_factor)

        elif shape_number == 3:
            draw_diagonal_45(start_pose, scaling_factor)
        
        elif shape_number == 4:
            draw_diagonal_135(start_pose, scaling_factor)

        elif shape_number == 5:
            draw_triangle(start_pose, scaling_factor)

        elif shape_number == 6:
            draw_L_shape(start_pose, scaling_factor)

        elif shape_number == 7:
            draw_square(start_pose, scaling_factor)

        elif shape_number == 8:
            draw_diamond(start_pose, scaling_factor)

    printing_goal_points()
    time.sleep(0.100)

    updated_flag_selectionArray = Float64MultiArray()
    updated_flag_selectionArray.data = []

    calculate_distance_difference()
    time.sleep(0.100)

    updated_flag_selectionArray = find_nearest_point_finally(shape_number) 
    time.sleep(0.100)

    select_final_goals(updated_flag_selectionArray, pub_PURPLE, pub_RED, pub_BLUE, pub_YELLOW)
    time.sleep(0.100)

#    time.sleep(0.200)   ## it's acting like:     rate.sleep()
#    publishing_generated_points(pub_PURPLE, pub_RED, pub_BLUE, pub_YELLOW)
#    flag_selectionArray.data = [-1, -1, -1, -1]


### ****************************************************************************
## ------------------------------------------------------------------------
## ------------------------- shapes to draw -------------------------------
## ------------------------------------------------------------------------
def draw_line(start_pose, scaling_factor):
    global step_size
    global X_f, Y_f, THETA_f, known_points
    global number_of_robots

    X_f.data = []
    Y_f.data = []
    THETA_f.data = []
    known_points.data = []

    X_f.data.append(start_pose.x)
    Y_f.data.append(start_pose.y)
    THETA_f.data.append(start_pose.theta)

    known_points.data.append(0)

    for i in range(number_of_robots - 1):
        new_x = X_f.data[i] + (scaling_factor * step_size)
        new_y = Y_f.data[i]
        new_theta = THETA_f.data[i]

        X_f.data.append(new_x)
        Y_f.data.append(new_y)
        THETA_f.data.append(new_theta)

        known_points.data.append(i + 1)
    
    time.sleep(0.010)   ## it's acting like:     rate.sleep()


### *****************************************
def draw_column(start_pose, scaling_factor):
    global step_size
    global X_f, Y_f, THETA_f, known_points
    global number_of_robots

    X_f.data = []
    Y_f.data = []
    THETA_f.data = []
    known_points.data = []

    X_f.data.append(start_pose.x)
    Y_f.data.append(start_pose.y)
    THETA_f.data.append(start_pose.theta)

    known_points.data.append(0)

    for i in range(number_of_robots - 1):
        new_x = X_f.data[i]
        new_y = Y_f.data[i] + (scaling_factor * step_size)
        new_theta = THETA_f.data[i]

        X_f.data.append(new_x)
        Y_f.data.append(new_y)
        THETA_f.data.append(new_theta)

        known_points.data.append(i + 1)
    
    time.sleep(0.010)   ## it's acting like:     rate.sleep()


### *****************************************
def draw_diagonal_45(start_pose, scaling_factor):
    global step_size
    global X_f, Y_f, THETA_f, known_points
    global number_of_robots

    X_f.data = []
    Y_f.data = []
    THETA_f.data = []
    known_points.data = []

    X_f.data.append(start_pose.x)
    Y_f.data.append(start_pose.y)
    THETA_f.data.append(start_pose.theta)

    known_points.data.append(0)

    distance_component_in_X = (scaling_factor * step_size * np.cos(45))
    distance_component_in_Y = (scaling_factor * step_size * np.sin(45))

    for i in range(number_of_robots - 1):
        new_x = X_f.data[i] + distance_component_in_X
        new_y = Y_f.data[i] + distance_component_in_Y
        new_theta = THETA_f.data[i]

        X_f.data.append(new_x)
        Y_f.data.append(new_y)
        THETA_f.data.append(new_theta)

        known_points.data.append(i + 1)
    
    time.sleep(0.010)   ## it's acting like:     rate.sleep()


### *****************************************
def draw_diagonal_135(start_pose, scaling_factor):
    global step_size
    global X_f, Y_f, THETA_f, known_points
    global number_of_robots

    X_f.data = []
    Y_f.data = []
    THETA_f.data = []
    known_points.data = []

    X_f.data.append(start_pose.x)
    Y_f.data.append(start_pose.y)
    THETA_f.data.append(start_pose.theta)

    known_points.data.append(0)

    distance_component_in_X = (scaling_factor * step_size * np.cos(45))
    distance_component_in_Y = (scaling_factor * step_size * np.sin(45))

    for i in range(number_of_robots - 1):
        new_x = X_f.data[i] - distance_component_in_X
        new_y = Y_f.data[i] + distance_component_in_Y
        new_theta = THETA_f.data[i]

        X_f.data.append(new_x)
        Y_f.data.append(new_y)
        THETA_f.data.append(new_theta)

        known_points.data.append(i + 1)
    
    time.sleep(0.010)   ## it's acting like:     rate.sleep()


### *****************************************
def draw_triangle(start_pose, scaling_factor):
    global step_size
    global X_f, Y_f, THETA_f, known_points
    global number_of_robots

    X_f.data = []
    Y_f.data = []
    THETA_f.data = []
    known_points.data = []

    THETA_f.data = [45.0, 315.0, 180.0]

    vertical_length = (3 * step_size) / 2
    distance_component_in_X = (scaling_factor / 3) * (vertical_length / 3) * 1.5
    distance_component_in_Y = scaling_factor * (step_size * np.cos(60 / 2)) * 1.2

    for i in range(number_of_robots):
        if i == 3:
            new_x = start_pose.x
            new_y = start_pose.y
            new_theta = start_pose.theta
            THETA_f.data.append(start_pose.theta)
        else:
            if i == 0:
                new_x = start_pose.x - distance_component_in_X
                new_y = start_pose.y - distance_component_in_Y

            elif i == 1:
                new_x = start_pose.x - distance_component_in_X
                new_y = start_pose.y + distance_component_in_Y

            elif i == 2:
                new_x = start_pose.x + distance_component_in_X
                new_y = start_pose.y

        X_f.data.append(new_x)
        Y_f.data.append(new_y)
        known_points.data.append(i)

    time.sleep(0.010)   ## it's acting like:     rate.sleep()


### *****************************************
def draw_L_shape(start_pose, scaling_factor):
    global step_size
    global X_f, Y_f, THETA_f, known_points
    global number_of_robots

    X_f.data = []
    Y_f.data = []
    THETA_f.data = [0.0, 0.0, 90.0, 180.0]
    known_points.data = []

    X_f.data.append(start_pose.x)
    Y_f.data.append(start_pose.y)

    known_points.data.append(0)

    distance_component_in_X = (scaling_factor * step_size * np.cos(0))
    distance_component_in_Y = (scaling_factor * step_size * np.sin(0))

    for i in range(number_of_robots - 1):
        if i == 2:
            distance_component_in_X = 0.0
            distance_component_in_Y = (scaling_factor * step_size * np.cos(0))

        new_x = X_f.data[i] + distance_component_in_X
        new_y = Y_f.data[i] + distance_component_in_Y

        X_f.data.append(new_x)
        Y_f.data.append(new_y)

        known_points.data.append(i + 1)
    
    time.sleep(0.010)   ## it's acting like:     rate.sleep()

### *****************************************
def draw_square(start_pose, scaling_factor):
    global step_size
    global X_f, Y_f, THETA_f, known_points
    global number_of_robots

    X_f.data = []
    Y_f.data = []
    THETA_f.data = [0.0, 90.0, 180.0, 270.0]
    known_points.data = []

    distance_component_in_X = (scaling_factor * step_size * np.cos(45))
    distance_component_in_Y = (step_size * np.sin(45))

    for i in range(number_of_robots):
        if i == 0:
            sign_x = -1
            sign_y = -1

        elif i == 1:
            sign_x = +1
            sign_y = -1

        elif i == 2:
            sign_x = +1
            sign_y = +1

        elif i == 3:
            sign_x = -1
            sign_y = +1

        new_x = start_pose.x + (sign_x * distance_component_in_X)
        new_y = start_pose.y + (sign_y * distance_component_in_Y)

        X_f.data.append(new_x)
        Y_f.data.append(new_y)

        known_points.data.append(i)

    time.sleep(0.010)   ## it's acting like:     rate.sleep()


### *****************************************
def draw_diamond(start_pose, scaling_factor):
    global step_size
    global X_f, Y_f, THETA_f
    global known_points
    X_f.data = [-1, -1, -1, -1]
    Y_f.data = [-1, -1, -1, -1]
    THETA_f.data = [-1, -1, -1, -1]
    known_points.data = [-1, -1, -1, -1]

    X_f.data[0] = start_pose.x - (step_size * scaling_factor)
    Y_f.data[0] = start_pose.y
    THETA_detected.data[0] = 0
    known_points.data[0] = 0

    X_f.data[1] = start_pose.x
    Y_f.data[1] = start_pose.y - (step_size * scaling_factor)
    THETA_detected.data[1] = 90
    known_points.data[1] = 1

    X_f.data[2] = start_pose.x + (step_size * scaling_factor)
    Y_f.data[2] = start_pose.y
    THETA_detected.data[2] = 180
    known_points.data[2] = 2

    X_f.data[3] = start_pose.x
    Y_f.data[3] = start_pose.y + (step_size * scaling_factor)
    THETA_detected.data[3] = 270
    known_points.data[3] = 3

    time.sleep(0.010)   ## it's acting like:     rate.sleep()


## ------------------------------------------------------------------------
## --------------------- formation calculations ---------------------------
## ------------------------------------------------------------------------
def publishing_generated_points(pub_PURPLE, pub_RED, pub_BLUE, pub_YELLOW):
    global activated_robots
    global goal_RED, goal_BLUE, goal_PURPLE, goal_YELLOW

    TOPIC_NAME = ' '
    GOAL = Pose2D()
    out_GOAL = Pose2D()

    out_GOAL.x = -5.0
    out_GOAL.y = -5.0
    out_GOAL.theta = 0.0

    for i in range(len(activated_robots.data)):
        TOPIC_NAME = ' '
        GOAL.x = 0.0
        GOAL.x = 0.0
        GOAL.theta = 0.0

        if activated_robots.data[i] != -1:
            if i == 0:
                TOPIC_NAME = 'formation_PURPLE'
                GOAL = goal_PURPLE

            elif i == 1:
                TOPIC_NAME = 'formation_RED'
                GOAL = goal_RED

            elif i == 2:
                TOPIC_NAME = 'formation_BLUE'
                GOAL = goal_BLUE
            
            else:
                TOPIC_NAME = 'formation_YELLOW'
                GOAL = goal_YELLOW

        else:
			if i == 0:
				TOPIC_NAME = 'formation_PURPLE'
				GOAL = out_GOAL

			elif i == 1:
				TOPIC_NAME = 'formation_RED'
				GOAL = out_GOAL

			elif i == 2:
				TOPIC_NAME = 'formation_BLUE'
				GOAL = out_GOAL

			else:
				TOPIC_NAME = 'formation_YELLOW'
				GOAL = out_GOAL

        time.sleep(0.050)

        publishing_object = rospy.Publisher(TOPIC_NAME, Pose2D, queue_size=10)
        publishing_object.publish(GOAL)
        time.sleep(0.100)   ## it's acting like:     rate.sleep()

### ****************************************************************************
def calculate_distance_difference():
    global distance_RED, distance_BLUE, distance_PURPLE, distance_YELLOW
    global X_f, Y_f, THETA_f
    global X_detected, Y_detected, THETA_detected
    global known_points

    distance_PURPLE.data = []
    distance_RED.data = []
    distance_BLUE.data = []
    distance_YELLOW.data = []

    print "\n"
    print "ROBOTS are calculating their distances ... \n"

    for i in range(len(activated_robots.data)):
        if activated_robots.data[i] != -1:
            if i == 0:
                distance_PURPLE.data = []
            elif i == 1:
                distance_RED.data = []
            elif i == 2:
                distance_BLUE.data = []
            elif i == 3:
                distance_YELLOW.data = []

            for j in range(len(X_f.data)):
                calculated_distance = 0.0
                diff_x = 0.0
                diff_y = 0.0

                diff_x = X_detected.data[i] - X_f.data[j]
                diff_y = Y_detected.data[i] - Y_f.data[j]
#                calculated_distance = np.hypot(diff_x, diff_y)
                calculated_distance = (diff_x**2 + diff_y**2)**0.5

                if i == 0:
                    distance_PURPLE.data.append(calculated_distance)
                elif i == 1:
                    distance_RED.data.append(calculated_distance)
                elif i == 2:
                    distance_BLUE.data.append(calculated_distance)
                elif i == 3:
                    distance_YELLOW.data.append(calculated_distance)

    time.sleep(0.010)   ## it's acting like:     rate.sleep()


### ****************************************************************************
def find_nearest_point_initially():
    global distance_RED, distance_BLUE, distance_PURPLE, distance_YELLOW
    global X_f, Y_f, THETA_f
    global goal_RED, goal_BLUE, goal_PURPLE, goal_YELLOW
    global activated_robots
    global flag_selectionArray

    min_RED = min_BLUE = min_YELLOW = min_PURPLE = 100

    flag_select_PURPLE = flag_select_RED = -1
    flag_select_BLUE = flag_select_YELLOW = -1

    print "ROBOTS are selecting INITIALLY their nearest goal ..."
    time.sleep(0.100)

    for i in range(len(activated_robots.data)):
        if activated_robots.data[i] != -1:
            if i == 0:
                for j in range(len(distance_PURPLE.data)):
                    if distance_PURPLE.data[j] < min_PURPLE:
                        min_PURPLE = distance_PURPLE.data[j]
                        flag_selectionArray.data[i] = j
            elif i == 1:
                for j in range(len(distance_RED.data)):
                    if distance_RED.data[j] < min_RED:
                        min_RED = distance_RED.data[j]
                        flag_selectionArray.data[i] = j
            elif i == 2:
                for j in range(len(distance_BLUE.data)):
                    if distance_BLUE.data[j] < min_BLUE:
                        min_BLUE = distance_BLUE.data[j]
                        flag_selectionArray.data[i] = j
            elif i == 3:
                for j in range(len(distance_YELLOW.data)):
                    if distance_YELLOW.data[j] < min_YELLOW:
                        min_YELLOW = distance_YELLOW.data[j]
                        flag_selectionArray.data[i] = j

    time.sleep(0.010)   ## it's acting like:     rate.sleep()


### ****************************************************************************
def not_taken_pointsArray():
    global flag_selectionArray
    global known_points
    global activated_robots

    not_selected_pointsArray = []
    not_found = 0

    for i in range(len(known_points.data)):
        not_found = 0
        for j in range(len(flag_selectionArray.data)):
            if flag_selectionArray.data[j] != -1:
                if known_points.data[i] != flag_selectionArray.data[j]:
                    continue
                else:
                    not_found = 1
                    
        if not_found == 0:
            not_selected_pointsArray.append(i)

    return not_selected_pointsArray



### ****************************************************************************
def solve_conflict(shape_number, i, j):
    global distance_RED, distance_BLUE, distance_PURPLE, distance_YELLOW
    global flag_selectionArray
    
    distance_conflictArray_i = Float64MultiArray()
    distance_conflictArray_j = Float64MultiArray()

    distance_conflictArray_i.data = []
    distance_conflictArray_j.data = []
    returned_not_selected_pointsArray = []

    print "SOLVING CONFLICT ... \n"

    time.sleep(0.100)

    if i == 0:
        distance_conflictArray_i = distance_PURPLE
    elif i == 1:
        distance_conflictArray_i = distance_RED
    elif i == 2:
        distance_conflictArray_i = distance_BLUE
    else:
        distance_conflictArray_i = distance_YELLOW

    if j == 0:
        distance_conflictArray_j = distance_PURPLE
    elif j == 1:
        distance_conflictArray_j = distance_RED
    elif j == 2:
        distance_conflictArray_j = distance_BLUE
    else:
        distance_conflictArray_j = distance_YELLOW

    returned_not_selected_pointsArray = not_taken_pointsArray()

    point_index = 0
    distanceArray_i = []
    distanceArray_j = []
    min_distance_i = 100
    min_distance_j = 100

    for iteration in range(len(returned_not_selected_pointsArray)):
        distanceArray_i.append(0.0)
        distanceArray_j.append(0.0)

    for iteration in range(len(returned_not_selected_pointsArray)):
        point_index = returned_not_selected_pointsArray[iteration]
        distanceArray_i[iteration] = distance_conflictArray_i.data[point_index]
        distanceArray_j[iteration] = distance_conflictArray_j.data[point_index]


    min_distance_i = min(distanceArray_i)
    min_distance_j = min(distanceArray_j)
        
    if min_distance_i < min_distance_j:
        if i == 0:
            for iteration in range(len(distance_PURPLE.data)):
                if distance_PURPLE.data[iteration] == min_distance_i:
                    point_index = iteration
                    break

        elif i == 1:
            for iteration in range(len(distance_RED.data)):
                if distance_RED.data[iteration] == min_distance_i:
                    point_index = iteration
                    break

        elif i == 2:
            for iteration in range(len(distance_BLUE.data)):
                if distance_BLUE.data[iteration] == min_distance_i:
                    point_index = iteration
                    break

        elif i == 3:
            for iteration in range(len(distance_YELLOW.data)):
                if distance_YELLOW.data[iteration] == min_distance_i:
                    point_index = iteration
                    break


        flag_selectionArray.data[i] = point_index
    else:
        flag_selectionArray.data[j] = point_index


    time.sleep(0.010)   ## it's acting like:     rate.sleep()
    updated_flag_selectionArray = flag_selectionArray
    return updated_flag_selectionArray


### ****************************************************************************
def find_nearest_point_finally(shape_number):
    global distance_RED, distance_BLUE, distance_PURPLE, distance_YELLOW
    global X_f, Y_f, THETA_f
    global goal_RED, goal_BLUE, goal_PURPLE, goal_YELLOW
    global flag_selectionArray

    find_nearest_point_initially()
    print "initial selected_points : \t", flag_selectionArray.data
    print "\n"
    updated_flag_selectionArray = Float64MultiArray()
    updated_flag_selectionArray = flag_selectionArray

    for i in range(len(flag_selectionArray.data)):
        current_point = flag_selectionArray.data[i]
        if current_point != -1:
            for j in range(len(flag_selectionArray.data)):
                if flag_selectionArray.data[j] != -1:
                    if j == i:
                        continue
                    elif current_point == flag_selectionArray.data[j]:
                        updated_flag_selectionArray = solve_conflict(shape_number, i, j)

    time.sleep(0.010)   ## it's acting like:     rate.sleep()
    print "final selected_points : \t", updated_flag_selectionArray.data
    return updated_flag_selectionArray


### ****************************************************************************
def select_final_goals(updated_flag_selectionArray, pub_PURPLE, pub_RED, pub_BLUE, pub_YELLOW):
    global X_f, Y_f, THETA_f
    global activated_robots
    global goal_RED, goal_BLUE, goal_PURPLE, goal_YELLOW
    global flag_selectionArray

    for i in range(len(flag_selectionArray.data)):
        time.sleep(0.030)
        if i == 0:
            flag_select_PURPLE = flag_selectionArray.data[i]
        elif i == 1:
            flag_select_RED = flag_selectionArray.data[i]
        elif i == 2:
            flag_select_BLUE = flag_selectionArray.data[i]
        else:
            flag_select_YELLOW = flag_selectionArray.data[i]

    print "\n"
    for i in range(len(activated_robots.data)):
        time.sleep(0.030)
        if activated_robots.data[i] != -1 and flag_selectionArray.data[i] != -1:
            if i == 0:
                print "PURPLE GOAL:"
                goal_PURPLE.x = X_f.data[flag_select_PURPLE]
                goal_PURPLE.y = Y_f.data[flag_select_PURPLE]
                goal_PURPLE.theta = THETA_f.data[flag_select_PURPLE]
                print goal_PURPLE, "\n"

            elif i == 1:
                print "RED GOAL:"
                goal_RED.x = X_f.data[flag_select_RED]
                goal_RED.y = Y_f.data[flag_select_RED]
                goal_RED.theta = THETA_f.data[flag_select_RED]
                print goal_RED, "\n"

            elif i == 2:
                print "BLUE GOAL: "
                goal_BLUE.x = X_f.data[flag_select_BLUE]
                goal_BLUE.y = Y_f.data[flag_select_BLUE]
                goal_BLUE.theta = THETA_f.data[flag_select_BLUE]
                print goal_BLUE, "\n"

            elif i == 3:
                print "YELLOW GOAL:"
                goal_YELLOW.x = X_f.data[flag_select_YELLOW]
                goal_YELLOW.y = Y_f.data[flag_select_YELLOW]
                goal_YELLOW.theta = THETA_f.data[flag_select_YELLOW]
                print goal_YELLOW, "\n"

    time.sleep(0.400)   ## it's acting like:     rate.sleep()
    publishing_generated_points(pub_PURPLE, pub_RED, pub_BLUE, pub_YELLOW)

    flag_selectionArray.data = [-1, -1, -1, -1]


## ------------------------------------------------------------------------
## --------------------------------- main ---------------------------------
## ------------------------------------------------------------------------
def main():
    global activated_robots, number_of_robots
    global goal_RED, goal_BLUE, goal_PURPLE, goal_YELLOW
    global selection_limits
    global selected_shape_name
    global global_dummy_counter
    global distance_PURPLE, distance_RED, distance_BLUE, distance_YELLOW

    rospy.init_node('formation_node', anonymous=False)
    rate = rospy.Rate(10)

    rospy.Subscriber('yellow', Pose2D, callback_YELLOW)
    rospy.Subscriber('red', Pose2D, callback_RED)
    rospy.Subscriber('blue', Pose2D, callback_BLUE)
    rospy.Subscriber('purple', Pose2D, callback_PURPLE)

    pub_PURPLE = rospy.Publisher('formation_PURPLE', Pose2D, queue_size=10)
    pub_RED = rospy.Publisher('formation_RED', Pose2D, queue_size=10)
    pub_BLUE = rospy.Publisher('formation_BLUE', Pose2D, queue_size=10)
    pub_YELLOW = rospy.Publisher('formation_YELLOW', Pose2D, queue_size=10)

    rate.sleep()
    time.sleep(0.500)

    dummy_counter = 0
    chosen_shape = 0
    regeneration_break = 1
    chosen_shape = -1
    final_flag_selectionArray = Float64MultiArray()
    final_flag_selectionArray.data = [-1, -1, -1, -1]
    time.sleep(0.500)


#    while not rospy.is_shutdown():
#        time.sleep(0.100)
#        print "activated_robots: ", activated_robots.data

#        time.sleep(0.500)
#        continue

    while not rospy.is_shutdown():
        time.sleep(0.500)
        check_active_robots(chosen_shape)
        rate.sleep()

        if dummy_counter == 0 and number_of_robots != 0:
            dummy_counter = dummy_counter + 1
            print "\n"
            print "- Welcome ... This is your first time here :)"
            global_dummy_counter = 0
            chosen_shape = select_shape()
            if chosen_shape >= selection_limits[0] and chosen_shape <= selection_limits[1]:
                draw_selected_shape(chosen_shape, pub_PURPLE, pub_RED, pub_BLUE, pub_YELLOW)
            else:
                print "NOT A VALID SHAPE ****************"
                check_active_robots(chosen_shape)

        elif dummy_counter > 0 and number_of_robots != 0 and chosen_shape != 0 and regeneration_break != 0:
            print "\n"
            if chosen_shape == 10:
                print "you have to choose a shape_number ..."
                chosen_shape = select_shape()
                if chosen_shape >= selection_limits[0] and chosen_shape <= selection_limits[1]:
                    draw_selected_shape(chosen_shape, pub_PURPLE, pub_RED, pub_BLUE, pub_YELLOW)
                else:
                    print "NOT A VALID SHAPE ****************"
                    check_active_robots(chosen_shape)


            else:
                answer = raw_input("do you want to form another shape? Y or N \t answer: ")

                if answer == "Y" or answer == "y":
                    global_dummy_counter = 0
                    chosen_shape = select_shape()
                    if chosen_shape >= selection_limits[0] and chosen_shape <= selection_limits[1]:
                        draw_selected_shape(chosen_shape, pub_PURPLE, pub_RED, pub_BLUE, pub_YELLOW)
                    else:
                        print "NOT A VALID SHAPE ****************"
                        check_active_robots(chosen_shape)

                elif answer == "N" or answer == "n":

                    if chosen_shape >= selection_limits[0] and chosen_shape <= selection_limits[1]:
                        print "........ WE WILL MAINTAIN THE PREVIOUS SHAPE ........"
                        regeneration_break = regenrate_shape(chosen_shape, pub_PURPLE, pub_RED, pub_BLUE, pub_YELLOW)

                    else:
                        print "THERE IS NO ENOUGH ROBOTS TO MAINTAIN THE SHAPE ..."
                        check_active_robots(chosen_shape)

                else:
                    print "THE NODE IS SHUTTING DOWN ... BYE *******************"
                    global_dummy_counter = 0
                    break

        else:
            if number_of_robots == 0:
                print "NO ROBOTS FOUND *********** WAITING ***********"

            elif chosen_shape == 0 or regeneration_break == 0:
                print "YOU HAVE QUITTED THE SELECTION PHASE **********"
                global_dummy_counter = 0
                try_again = raw_input("do you want to try again? Y or N \t answer: ")

                if try_again == "N" or try_again == "n":
                    print "THE NODE IS SHUTTING DOWN ... BYE BYE *******************"
                    global_dummy_counter = 0
                    break

                else:
                    chosen_shape = 10
                    regeneration_break = 1
                    continue



## --------------------------------- entry point ---------------------------
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print "ERROR !!!"
