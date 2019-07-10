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

yellow_circle = [0,0]
red_circle = [0,0]

big_red_circle = [0,0]
small_red_circle = [0,0]

big_purple_circle = [0,0]
small_purple_circle = [0,0]

big_blue_circle = [0,0]
small_blue_circle = [0,0]

big_yellow_circle = [0,0]
small_yellow_circle = [0,0]

pt1 = (281,83)
pt2 = (1029,92)
pt3 = (141,664)
pt4 = (1165,667)
pts1 = np.float32([[pt1],[pt2],[pt3],[pt4]])
pts2 = np.float32([[0,0],[800,0],[0,600],[800,600]])



# RED FIX 
##obstacles fucked up the red color
red_upper = np.array([165, 200, 255])
red_lower = np.array([80, 110, 175])

blue_upper = np.array([255, 220, 150])
blue_lower = np.array([180,130, 80])

black_upper = np.array([130, 180, 110])
black_lower = np.array([60, 60, 50])

yellow_upper = np.array([200, 255, 255])
yellow_lower = np.array([140, 180, 195])

purple_upper = np.array([255, 165, 190])
purple_lower = np.array([120, 90, 135])


def callback(msg):
	global red_robot,yellow_robot,blue_robot,purple_robot
	global big_red_circle,small_red_circle,big_yellow_circle,small_yellow_circle,big_purple_circle,small_purple_circle,big_blue_circle,small_blue_circle
	try:
		frame = bridge.imgmsg_to_cv2(msg, "bgr8")
	except CvBridgeError as e:
		print(e)

	cropped_image = cv2.getPerspectiveTransform(pts1,pts2)
	final_image = cv2.warpPerspective(frame,cropped_image,(800,600))
##############################################################################################################################
###############################COLOR DETECTION################################################################################
##############################################################################################################################
	hsv = cv2.cvtColor(final_image, cv2.COLOR_BGR2HSV)

	red_mask = cv2.inRange(final_image,red_lower,red_upper)
	red_mask = cv2.erode(red_mask, None, iterations=2)
	red_mask = cv2.dilate(red_mask, None, iterations=2)

	blue_mask = cv2.inRange(final_image,blue_lower,blue_upper)
	blue_mask = cv2.erode(blue_mask, None, iterations=2)
	blue_mask = cv2.dilate(blue_mask, None, iterations=2)

	black_mask = cv2.inRange(final_image,black_lower,black_upper)
	black_mask = cv2.erode(black_mask, None, iterations=2)
	black_mask = cv2.dilate(black_mask, None, iterations=2)

	yellow_mask = cv2.inRange(final_image,yellow_lower,yellow_upper)
	yellow_mask = cv2.erode(yellow_mask, None, iterations=2)
	yellow_mask = cv2.dilate(yellow_mask, None, iterations=2)

	purple_mask = cv2.inRange(final_image,purple_lower,purple_upper)
	purple_mask = cv2.erode(purple_mask, None, iterations=2)
	purple_mask = cv2.dilate(purple_mask, None, iterations=2)


	res_red = cv2.bitwise_and(final_image, final_image, mask=red_mask)
	res_blue = cv2.bitwise_and(final_image, final_image, mask=blue_mask)
	res_black = cv2.bitwise_and(final_image, final_image, mask=black_mask)
	res_yellow = cv2.bitwise_and(final_image, final_image, mask=yellow_mask)
	res_purple = cv2.bitwise_and(final_image, final_image, mask=purple_mask)
	
	yellow = cv2.medianBlur(res_yellow, 5)
	red = cv2.medianBlur(res_red, 5)
	purple = cv2.medianBlur(res_purple, 5)
	blue = cv2.medianBlur(res_blue, 5)
	black = cv2.medianBlur(res_black, 5)
	# cv2.imshow('RED',red)
	# cv2.imshow('PURPLE',purple)
	# cv2.imshow('BLUE',blue)
	# cv2.imshow('BLACK',black)
	# cv2.imshow('YELLOW',yellow)
##############################################################################################################################
##############################################################################################################################
##############################################################################################################################
##############################################################################################################################
###################################################RED CIRCLE DETECTON########################################################
##############################################################################################################################
##############################################################################################################################
##############################################################################################################################
##############################################################################################################################
	red_robot.x = -1
	red_robot.y = -1
	red_robot.theta = 0
	
	gray_r = cv2.cvtColor(red, cv2.COLOR_BGR2GRAY)
	gray_r = cv2.GaussianBlur(gray_r,(11,11),0)  
	rows = gray_r.shape[0]
	circles = cv2.HoughCircles(gray_r, cv2.HOUGH_GRADIENT, 1, rows / 3, param1=100, param2=5, minRadius=18, maxRadius=19)
	if circles is not None:
		circles = np.uint16(np.around(circles))
		for i in circles[0, :]:

				center = (i[0], i[1])
		    # circle center
				#cv2.circle(final_image, center, 1, (100, 100, 100), 1)
				# red_robot.x= i[0]/30
				# red_robot.y= i[1]/30
				big_red_circle = [np.float(i[0]/3),np.float(i[1]/3)]
		    # circle outline
				radius = i[2]
				cv2.circle(final_image, center, radius, (0, 0, 255), 2)

			

	circles = cv2.HoughCircles(gray_r, cv2.HOUGH_GRADIENT, 1, rows / 3, param1=70, param2=10, minRadius=9, maxRadius=10)
	if circles is not None:
		circles = np.uint16(np.around(circles))
		for i in circles[0, :]:
			# if red_filter == 0:
			center = (i[0], i[1])
		    # circle center
			# cv2.circle(final_image, center, 1, (100, 100, 100), 1)
			small_red_circle = [np.float(i[0]/3),np.float(i[1]/3)]
	    # circle outline
			#print ('Red Robot = ',i[0]/30,i[1]/30)
			radius = i[2]
			cv2.circle(final_image, center, radius, (0, 0, 255), 2)
			# red_filter = 1
			# else:
				# red_filter = 0
	# if (small_red_circle[0] - big_red_circle[0]) == 0 or (small_red_circle[1] - big_red_circle[1]) == 0 or (big_red_circle[0] - small_red_circle[0]) == 0 or (big_red_circle[1] - small_red_circle[1]) == 9.0:
		# pass
	# else:
	rad = math.atan2(np.int16(big_red_circle[1]) - np.int16(small_red_circle[1]),np.int16((big_red_circle[0]) - np.int16(small_red_circle[0])))
	if (np.int16(rad * (180/3.14159265)) + 180) == 360:
		red_robot.theta = 0
	else:
		red_robot.theta = np.int16(rad * (180/3.14159265)) + 180

	if big_red_circle[0]>small_red_circle[0]:
		
		red_robot.x = abs(big_red_circle[0] - ((big_red_circle[0]-small_red_circle[0])/2) )/10

	else:
		red_robot.x = abs(big_red_circle[0] - ((big_red_circle[0]-small_red_circle[0])/2) )/10


	if big_red_circle[1]>small_red_circle[1]:
		
		red_robot.y = abs(big_red_circle[1] - abs((big_red_circle[1]-small_red_circle[1])/2) )/10

	else:
		red_robot.y = abs(big_red_circle[1] + abs((big_red_circle[1]-small_red_circle[1])/2) )/10


	if int(red_robot.x) <= 1 or int(red_robot.y) <= 1:
		red_robot.x = -5
		red_robot.y = -5
		red_robot.theta = 0
		red_pub.publish(red_robot)

	while red_robot.theta%5 !=0 :
		red_robot.theta = red_robot.theta-1

	# red_robot.theta = 0
	# print ("Red Theta = ",red_robot.theta)
	# print

	# center = (int(red_robot.x*30),int(red_robot.y*30))
	# cv2.circle(final_image, center, 5, (0, 0, 255), 6)

	# print center
	# pts_red.append(center)
	# cv2.circle(final_image, center, int(circle_raduis*30), (0, 0, 255), 5)
	# cv2.circle(final_image, center, 5, (0, 0, 255), 8)

	# for i in range(1, len(pts_red)):
	# 	cv2.line(final_image, pts_red[i], pts_red[i-1], (0, 0, 255), 5)


##############################################################################################################################
##############################################################################################################################
##############################################################################################################################
##############################################################################################################################
#################################################YELLOW CIRCLE DETECTON#######################################################
##############################################################################################################################
##############################################################################################################################
##############################################################################################################################
##############################################################################################################################

	yellow_robot.x = -1
	yellow_robot.y = -1
	yellow_robot.theta = 0


	gray_r = cv2.cvtColor(yellow, cv2.COLOR_BGR2GRAY)
	gray_r = cv2.GaussianBlur(gray_r,(11,11),0)  
	rows = gray_r.shape[0]
	circles = cv2.HoughCircles(gray_r, cv2.HOUGH_GRADIENT, 1, rows / 3, param1=100, param2=10, minRadius=17, maxRadius=20)
	if circles is not None:
		circles = np.uint16(np.around(circles))
		for i in circles[0, :]:

				center = (i[0], i[1])
		    # circle center
				#cv2.circle(final_image, center, 1, (0, 255, 255), 1)
				big_yellow_circle = [np.float(i[0]/3),np.float(i[1]/3)]
		    # circle outline
				radius = i[2]
				cv2.circle(final_image, center, radius, (0, 255, 255), 2)

			

	circles = cv2.HoughCircles(gray_r, cv2.HOUGH_GRADIENT, 1, rows / 3, param1=70, param2=10, minRadius=7, maxRadius=10)
	if circles is not None:
		circles = np.uint16(np.around(circles))
		for i in circles[0, :]:
			center = (i[0], i[1])
		    # circle center
			#cv2.circle(final_image, center, 1, (0, 255, 255), 1)
			small_yellow_circle = [np.float(i[0]/3),np.float(i[1]/3)]
	    # circle outline
			radius = i[2]
			cv2.circle(final_image, center, radius, (0, 255, 255), 2)
	rad = math.atan2(np.int16(big_yellow_circle[1]) - np.int16(small_yellow_circle[1]),np.int16((big_yellow_circle[0]) - np.int16(small_yellow_circle[0])))
	if (np.int16(rad * (180/3.14159265)) + 180) == 360:
		yellow_robot.theta = 0
	else:
		yellow_robot.theta = np.int16(rad * (180/3.14159265)) + 180
	yellow_robot.x = abs(big_yellow_circle[0])/10
	yellow_robot.y = abs(big_yellow_circle[1])/10
	# print ("X_yellow = ",yellow_robot.x)
	# print ("Y_yellow = ",yellow_robot.y)

	if big_yellow_circle[0]>small_yellow_circle[0]:
		
		yellow_robot.x = abs(big_yellow_circle[0] - ((big_yellow_circle[0]-small_yellow_circle[0])/2) )/10

	else:
		yellow_robot.x = abs(big_yellow_circle[0] - ((big_yellow_circle[0]-small_yellow_circle[0])/2) )/10


	if big_yellow_circle[1]>small_yellow_circle[1]:
		
		yellow_robot.y = abs(big_yellow_circle[1] - abs((big_yellow_circle[1]-small_yellow_circle[1])/2) )/10

	else:
		yellow_robot.y = abs(big_yellow_circle[1] + abs((big_yellow_circle[1]-small_yellow_circle[1])/2) )/10

	if int(yellow_robot.x) <= 1 or int(yellow_robot.y) <= 1:
		yellow_robot.x = -5
		yellow_robot.y = -5
		yellow_robot.theta = 0
		yellow_pub.publish(yellow_robot)

	while yellow_robot.theta%5 !=0 :
		yellow_robot.theta = yellow_robot.theta-1

	# print ("Yellow Theta= ",yellow_robot.theta)

	# center = (int(yellow_robot.x*30),int(yellow_robot.y*30))
	# cv2.circle(final_image, center, 5, (0, 255, 255), 6)

	# pts_yellow.append(center)
	# cv2.circle(final_image, center, int(circle_raduis*30), (0, 255, 255), 5)
	# for i in range(1, len(pts_yellow)):
	# 	cv2.line(final_image, pts_yellow[i], pts_yellow[i-1], (0, 255, 255), 5)



##############################################################################################################################
##############################################################################################################################
##############################################################################################################################
##############################################################################################################################
##################################################PURPLE CIRCLE DETECTON#######################################################
##############################################################################################################################
##############################################################################################################################
##############################################################################################################################
##############################################################################################################################
	purple_robot.x = -1
	purple_robot.y = -1
	purple_robot.theta = 0


	gray_r = cv2.cvtColor(purple, cv2.COLOR_BGR2GRAY)
	gray_r = cv2.GaussianBlur(gray_r,(11,11),0)  
	rows = gray_r.shape[0]
	circles = cv2.HoughCircles(gray_r, cv2.HOUGH_GRADIENT, 1, rows / 3, param1=100, param2=5, minRadius=16, maxRadius=18)
	if circles is not None:
		circles = np.uint16(np.around(circles))
		for i in circles[0, :]:

				center = (i[0], i[1])
		    # circle center
				#cv2.circle(final_image, center, 1, (100, 100, 100), 1)
				big_purple_circle = [np.float(i[0]/3),np.float(i[1]/3)]
		    # circle outline
				radius = i[2]
				cv2.circle(final_image, center, radius, (214, 0, 148), 2)

			

	circles = cv2.HoughCircles(gray_r, cv2.HOUGH_GRADIENT, 1, rows / 3, param1=70, param2=10, minRadius=6, maxRadius=9)
	if circles is not None:
		circles = np.uint16(np.around(circles))
		for i in circles[0, :]:
			center = (i[0], i[1])
		    # circle center
			#cv2.circle(final_image, center, 1, (100, 100, 100), 1)
			small_purple_circle = [np.float(i[0]/3),np.float(i[1]/3)]
	    # circle outline
			radius = i[2]
			cv2.circle(final_image, center, radius, (214, 0, 148), 2)
	rad = math.atan2(np.int16(big_purple_circle[1]) - np.int16(small_purple_circle[1]),np.int16((big_purple_circle[0]) - np.int16(small_purple_circle[0])))
	if (np.int16(rad * (180/3.14159265)) + 180) == 360:
		purple_robot.theta = 0
	else:
		purple_robot.theta = np.int16(rad * (180/3.14159265)) + 180
	# purple_robot.x = abs(big_purple_circle[0])/10
	# purple_robot.y = abs(big_purple_circle[1])/10

	if big_purple_circle[0]>small_purple_circle[0]:
		
		purple_robot.x = abs(big_purple_circle[0] - ((big_purple_circle[0]-small_purple_circle[0])/2) )/10

	else:
		purple_robot.x = abs(big_purple_circle[0] - ((big_purple_circle[0]-small_purple_circle[0])/2) )/10


	if big_purple_circle[1]>small_purple_circle[1]:
		
		purple_robot.y = abs(big_purple_circle[1] - abs((big_purple_circle[1]-small_purple_circle[1])/2) )/10

	else:
		purple_robot.y = abs(big_purple_circle[1] + abs((big_purple_circle[1]-small_purple_circle[1])/2) )/10

	if int(purple_robot.x) <= 1 or int(purple_robot.y) <= 1:
		purple_robot.x = -5
		purple_robot.y = -5
		purple_robot.theta = 0
		purple_pub.publish(purple_robot)		


	while purple_robot.theta%5 !=0 :
		purple_robot.theta = purple_robot.theta-1

	# center = (int(purple_robot.x*30),int(purple_robot.y*30))

	# pts_purple.append(center)
	# cv2.circle(final_image, center, 5, (214, 0, 148), 6)

	# cv2.circle(final_image, center, int(circle_raduis*30), (214, 0, 148), 5)
	# for i in range(1, len(pts_purple)):
	# 	cv2.line(final_image, pts_purple[i], pts_purple[i-1], (241, 0, 148), 5)

	# print ("purple Theta= ",purple_robot.theta)
##############################################################################################################################
##############################################################################################################################
###################################################BLUE CIRCLE DETECTON#######################################################
##############################################################################################################################
##############################################################################################################################


	gray_r = cv2.cvtColor(blue, cv2.COLOR_BGR2GRAY)
	gray_r = cv2.GaussianBlur(gray_r,(11,11),0)  
	rows = gray_r.shape[0]
	circles = cv2.HoughCircles(gray_r, cv2.HOUGH_GRADIENT, 1, rows / 3, param1=100, param2=5, minRadius=18, maxRadius=19)
	if circles is not None:
		circles = np.uint16(np.around(circles))
		for i in circles[0, :]:

				center = (i[0], i[1])
		    # circle center
				# cv2.circle(final_image, center, 1, (255, 0, 0), 1)
				# red_robot.x= i[0]/30
				# red_robot.y= i[1]/30
				big_blue_circle = [np.float(i[0]/3),np.float(i[1]/3)]
		    # circle outline
				radius = i[2]
				cv2.circle(final_image, center, radius, (255, 0, 0), 2)

			

	circles = cv2.HoughCircles(gray_r, cv2.HOUGH_GRADIENT, 1, rows / 3, param1=70, param2=10, minRadius=8, maxRadius=10)
	if circles is not None:
		circles = np.uint16(np.around(circles))
		for i in circles[0, :]:
			# if red_filter == 0:
			center = (i[0], i[1])
		    # circle center
			# cv2.circle(final_image, center, 1, (255, 0, 0), 1)
			small_blue_circle = [np.float(i[0]/3),np.float(i[1]/3)]
	    # circle outline
			#print ('Red Robot = ',i[0]/30,i[1]/30)
			radius = i[2]
			cv2.circle(final_image, center, radius, (255, 0, 0), 2)
	rad = math.atan2(np.int16(big_blue_circle[1]) - np.int16(small_blue_circle[1]),np.int16((big_blue_circle[0]) - np.int16(small_blue_circle[0])))
	if (np.int16(rad * (180/3.14159265)) + 180) == 360:
		blue_robot.theta = 0
	else:
		blue_robot.theta = np.int16(rad * (180/3.14159265)) + 180

	if big_blue_circle[0]>small_blue_circle[0]:
		
		blue_robot.x = abs(big_blue_circle[0] - ((big_blue_circle[0]-small_blue_circle[0])/2) )/10

	else:
		blue_robot.x = abs(big_blue_circle[0] - ((big_blue_circle[0]-small_blue_circle[0])/2) )/10


	if big_blue_circle[1]>small_blue_circle[1]:
		
		blue_robot.y = abs(big_blue_circle[1] - abs((big_blue_circle[1]-small_blue_circle[1])/2) )/10

	else:
		blue_robot.y = abs(big_blue_circle[1] + abs((big_blue_circle[1]-small_blue_circle[1])/2) )/10

	if int(blue_robot.x) <= 1 or int(blue_robot.y) <= 1:
		blue_robot.x = -5
		blue_robot.y = -5
		blue_robot.theta = 0
		blue_pub.publish(blue_robot)		


#	red_robot.x = abs(big_red_circle[0])/10
	# red_robot.y = abs(big_red_circle[1])/10
	# print ("X_blue = ",blue_robot.x)
	# print ("Y_blue = ",blue_robot.y)


	while blue_robot.theta%5 !=0 :
		blue_robot.theta = blue_robot.theta-1

	# print ("blue Theta = ",blue_robot.theta)
	# center = (int(blue_robot.x*30),int(blue_robot.y*30))

	# pts_blue.append(center)
	# cv2.circle(final_image, center, 5, (255, 0, 0), 6)
	# for i in range(1, len(pts_blue)):
	# 	cv2.line(final_image, pts_blue[i], pts_blue[i-1], (255, 0, 0), 5)




#####################################################################################
#########################BLACK CIRCLES DETECTION#####################################
#####################################################################################
	gray_b = cv2.cvtColor(black, cv2.COLOR_BGR2GRAY)
	gray_b = cv2.GaussianBlur(gray_b,(11,11),0)  
	rows = gray_b.shape[0]
	circles = cv2.HoughCircles(gray_b, cv2.HOUGH_GRADIENT, 1, rows / 16, param1=90, param2=27, minRadius=22, maxRadius=24)
	counter = 0
	obst_1.x=0
	obst_1.y=0
	obst_1.theta = 0

	obst_2.x=0
	obst_2.y=0
	obst_2.theta = 0


	obst_3.x=0
	obst_3.y=0
	obst_3.theta = 0


	obst_4.x =0
	obst_4.y =0
	obst_4.theta = 0

	
	if circles is not None:
		circles = np.uint16(np.around(circles))
		for i in circles[0, :]:
			center = (i[0], i[1])
			if counter == 0 :
				obst_1.x=i[0]/30
				obst_1.y=i[1]/30
				counter = counter + 1

			if counter == 1 :
				obst_2.x=i[0]/30
				obst_2.y=i[1]/30
				counter = counter + 1

			if counter == 2 :
				obst_3.x=i[0]/30
				obst_3.y=i[1]/30
				counter = counter + 1

			else:
				obst_4.x =i[0]/30
				obst_4.y =i[1]/30
				counter = 0
			
           # circle center
			#cv2.circle(final_image, center, 1, (100, 100, 100), 1)
            # circle outline
			radius = i[2]
			cv2.circle(final_image, center, radius, (0, 0, 0), 2)


	if red_robot.x !=-5 and red_robot.y !=-5 and red_robot.theta != 0:
		red_pub.publish(red_robot)
	if yellow_robot.x !=-5 and yellow_robot.y !=-5 and yellow_robot.theta != 0:
		yellow_pub.publish(yellow_robot)
	if purple_robot.x !=-5 and purple_robot.y !=-5 and purple_robot.theta != 0:
		purple_pub.publish(purple_robot)
	if blue_robot.x !=-5 and blue_robot.y !=-5 and blue_robot.theta != 0:	
		blue_pub.publish(blue_robot)
	Obst_1.publish(obst_1)
	Obst_2.publish(obst_2)
	Obst_1.publish(obst_3)
	Obst_2.publish(obst_4)
	# cv2.imshow("final_image",final_image)
	#print (pub_counter)
	cv2.waitKey(3)
#####################################################################################
#####################################################################################
#####################################################################################
#####################################################################################
#####################################################################################
	try:
		image_pub.publish(bridge.cv2_to_imgmsg(final_image, "bgr8"))
	except CvBridgeError as e:
		print(e)

if __name__== '__main__':
	print ("Code is running!!")
	rospy.init_node('image_listener')
	image_topic = "/usb_cam/image_raw"
	image_pub = rospy.Publisher("image_topic_2",Image,queue_size=1)
	red_pub = rospy.Publisher("red",Pose2D,queue_size=1)
	yellow_pub = rospy.Publisher("yellow",Pose2D,queue_size=1)
	purple_pub = rospy.Publisher("purple",Pose2D,queue_size=1)
	blue_pub = rospy.Publisher("blue",Pose2D,queue_size=1)
	Obst_1 = rospy.Publisher("obst_1",Pose2D,queue_size=1)
	Obst_2 = rospy.Publisher("obst_2",Pose2D,queue_size=1)
	Obst_3 = rospy.Publisher("obst_3",Pose2D,queue_size=1)
	Obst_4 = rospy.Publisher("obst_4",Pose2D,queue_size=1)
	rospy.Subscriber(image_topic, Image, callback)
	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
