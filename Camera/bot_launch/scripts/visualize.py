#! /usr/bin/python
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from collections import deque
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int16
from geometry_msgs.msg import Pose2D
import argparse
import math
red_goal_x = Float64MultiArray()
red_goal_y = Float64MultiArray()
blue_goal_x = Float64MultiArray()
blue_goal_y = Float64MultiArray()
yellow_goal_x = Float64MultiArray()
yellow_goal_y = Float64MultiArray()
purple_goal_x = Float64MultiArray()
purple_goal_y = Float64MultiArray()

red_goal_x = Float64MultiArray()
red_goal_y = Float64MultiArray()
blue_goal_x = Float64MultiArray()
blue_goal_y = Float64MultiArray()
yellow_goal_x = Float64MultiArray()
yellow_goal_y = Float64MultiArray()
purple_goal_x = Float64MultiArray()
purple_goal_y = Float64MultiArray()

bridge = CvBridge()

circle_raduis = 2.4

red_robot = Pose2D()
yellow_robot = Pose2D()
purple_robot = Pose2D()
blue_robot = Pose2D()
obst_1 = Pose2D()
obst_2 = Pose2D()
obst_3 = Pose2D()
obst_4 = Pose2D()

red_goal = Pose2D()
purple_goal = Pose2D()
blue_goal = Pose2D()
yellow_goal = Pose2D()

def callback(msg):
	global red_goal,yellow_goal,blue_goal,purple_goal
	global red_goal_x,red_goal_y,blue_goal_x,blue_goal_y,yellow_goal_x,yellow_goal_y,purple_goal_x,purple_goal_y

	try:
		final_image = bridge.imgmsg_to_cv2(msg, "bgr8")
	except CvBridgeError as e:
		print(e)
	if red_robot.x > 3:
		cv2.circle(final_image, (int(30*red_robot.x),int(30*red_robot.y)), int(circle_raduis*30), (0, 0, 255), 2)
	if blue_robot.x > 3:
		cv2.circle(final_image, (int(30*blue_robot.x),int(30*blue_robot.y)), int(circle_raduis*30), (255, 0, 0), 2)
	if purple_robot.x > 3:
		cv2.circle(final_image, (int(30*purple_robot.x),int(30*purple_robot.y)), int(circle_raduis*30), (241, 0, 148), 2)
	if yellow_robot.x > 3:
		cv2.circle(final_image, (int(30*yellow_robot.x),int(30*yellow_robot.y)), int(circle_raduis*30), (0, 255, 255), 2)

	cv2.circle(final_image, (int(30*red_goal.x),int(30*red_goal.y)), 20, (0, 0, 255), 2)
	cv2.circle(final_image, (int(30*blue_goal.x),int(30*blue_goal.y)), 20, (255, 0, 0), 2)
	cv2.circle(final_image, (int(30*purple_goal.x),int(30*purple_goal.y)), 20, (241, 0, 148), 2)
	cv2.circle(final_image, (int(30*yellow_goal.x),int(30*yellow_goal.y)), 20, (0, 255, 255), 2)

	for i in range(len(red_goal_x.data)):
		cv2.circle(final_image, (int(30*red_goal_x.data[i]),int(30*red_goal_y.data[i])), 5, (0, 0, 255), 3)

	for i in range(len(blue_goal_x.data)):
		cv2.circle(final_image, (int(30*blue_goal_x.data[i]),int(30*blue_goal_y.data[i])), 5, (255, 0, 0), 3)

	for i in range(len(yellow_goal_x.data)):
		cv2.circle(final_image, (int(30*yellow_goal_x.data[i]),int(30*yellow_goal_y.data[i])), 5, (0, 255, 255), 3)

	for i in range(len(purple_goal_x.data)):
		cv2.circle(final_image, (int(30*purple_goal_x.data[i]),int(30*purple_goal_y.data[i])), 5, (241, 0, 148), 3)

	try:
		image_pub.publish(bridge.cv2_to_imgmsg(final_image, "bgr8"))
	except CvBridgeError as e:
		print(e)

def red(data):
	global red_robot
	# print "red_callback"
	red_robot = data
def blue(data):
	global blue_robot
	# print "red_callback"
	blue_robot = data
def yellow(data):
	global yellow_robot
	# print "red_callback"
	yellow_robot = data
def purple(data):
	global purple_robot
	# print "red_callback"
	purple_robot = data


def red_callback(data):
	global red_goal
	# print "red_callback"
	red_goal = data

def purple_callback(data):
	# print "purple_callback"
	global purple_goal
	purple_goal = data

def yellow_callback(data):
	# print "yellow_callback"
	global yellow_goal
	yellow_goal = data

def blue_callback(data):
	# print "blue_callback"
	global blue_goal
	blue_goal = data


def red_x_goal(data):
	global red_goal_x
	red_goal_x = data
def red_y_goal(data):
	global red_goal_y
	red_goal_y = data

def blue_x_goal(data):
	global blue_goal_x
	blue_goal_x = data
def blue_y_goal(data):
	global blue_goal_y
	blue_goal_y = data

def yellow_x_goal(data):
	global yellow_goal_x
	yellow_goal_x = data
def yellow_y_goal(data):
	global yellow_goal_y
	yellow_goal_y = data

def purple_x_goal(data):
	global purple_goal_x
	purple_goal_x = data
def purple_y_goal(data):
	global purple_goal_y
	purple_goal_y = data

if __name__== '__main__':
	print ("Code is running!!")
	rospy.init_node('image_visualizer')
	image_topic = "/image_raw"
	image_pub = rospy.Publisher("image",Image,queue_size=1)

	rospy.Subscriber(image_topic, Image, callback)
	rospy.Subscriber("formation_RED", Pose2D, red_callback)
	rospy.Subscriber("formation_BLUE", Pose2D, blue_callback)
	rospy.Subscriber("formation_PURPLE", Pose2D, purple_callback)
	rospy.Subscriber("formation_YELLOW", Pose2D, yellow_callback)

	rospy.Subscriber("red", Pose2D, red)
	rospy.Subscriber("blue", Pose2D, blue)
	rospy.Subscriber("purple", Pose2D, purple)
	rospy.Subscriber("yellow", Pose2D, yellow)

	rospy.Subscriber("X_RED", Float64MultiArray, red_x_goal)
	rospy.Subscriber("Y_RED", Float64MultiArray, red_y_goal)
	rospy.Subscriber("X_YELLOW", Float64MultiArray, yellow_x_goal)
	rospy.Subscriber("Y_YELLOW", Float64MultiArray, yellow_y_goal)
	rospy.Subscriber("X_BLUE", Float64MultiArray, blue_x_goal)
	rospy.Subscriber("Y_BLUE", Float64MultiArray, blue_y_goal)
	rospy.Subscriber("X_PURPLE", Float64MultiArray, purple_x_goal)
	rospy.Subscriber("Y_PURPLE", Float64MultiArray, purple_y_goal)

	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
