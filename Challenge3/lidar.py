import cv2
import rospy as rp
import numpy as np
import math
import time
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
bridge = CvBridge()

pub = rp.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size = 10)
pub1 = rp.Publisher('CVOutput', String, queue_size = 10)
image_right = []

#How close can an object be infront of us before we stop
full_stop = 0.3
#Determines what portion of the course we are in
# 0 = Initial Alignment
# 1 = Parallel
# 2 = Weave
state = 2
#Remember what side we are weaving on
# 1 = Right
# 2 = Left
substate = 1
#time passed since last substate change
time_start = time.time()
minTime = 2.0

orbit_dist = 1.2

check = False

def drive(speed, steering):
	drive_msg_stamped = AckermannDriveStamped()
	drive_msg = AckermannDrive()
       	drive_msg.speed = speed
        drive_msg.steering_angle = steering
        drive_msg.acceleration = 0
        drive_msg.jerk = 0
        drive_msg.steering_angle_velocity = 0
	drive_msg_stamped.drive = drive_msg
	pub.publish(drive_msg_stamped)

def zedCam(data):
	global state, substate, check
	cv_img = bridge.imgmsg_to_cv2(data, desired_encoding = "passthrough")

	# Crop the image
	#crop_img = cv_img[250:675, 50:700]
	
	# Uncomment if you want to use original image
	crop_img = cv_img

	kernel = np.ones((5,5),np.uint8)
	# Gaussian Blur image
	crop_img = cv2.GaussianBlur(crop_img, (11,11), 0)

	# Convert RGB image to HSV image
	hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

	# Grayscale image
	gray_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)

	# Define range of blue color in HSV
	lower_blue = np.array([10, 235, 80], dtype = "uint8")
	upper_blue = np.array([30, 255, 255], dtype = "uint8")

	# Mask of blue in the image
	blue_mask = cv2.inRange(hsv_img, lower_blue, upper_blue)
	blue_mask = cv2.erode(blue_mask, kernel, iterations = 2)
	blue_mask = cv2.dilate(blue_mask, kernel, iterations = 2)
	blue_mask = cv2.dilate(blue_mask, kernel, iterations = 2)
	blue_mask = cv2.erode(blue_mask, kernel, iterations = 2)

	# Display the resulting frame
	cv2.imshow('mask', blue_mask)
	cv2.waitKey(1)

	# Color thresholding
	ret,thresh = cv2.threshold(blue_mask, 10, 255, cv2.THRESH_BINARY)

	# Find the contours of the frame
	_,contours,hierarchy = cv2.findContours(thresh.copy(), 1, cv2.CHAIN_APPROX_NONE)

	# Find the biggest contour
	if len(contours) > 0:
		c = max(contours, key = cv2.contourArea)
		M = cv2.moments(c)

	cx = int(M['m10']/M['m00'])
	cy = int(M['m01']/M['m00'])

	cv2.line(crop_img, (cx,0), (cx,720), (255,0,0), 1)
	cv2.line(crop_img, (0,cy), (1280,cy), (255,0,0), 1)
	cv2.drawContours(crop_img, contours, -1, (0,0,255), 1)

	print("CY: ", cy)
	if(cy < 95):
		if(substate == 1):
			if(check == True):
				substate =2
				check = False
		elif(substate == 2):
			if(check == True):
				substate =1
				check = False
	if(cy >= 95):
		if(check == False):
			check = True
		pub1.publish(str(cx))

	# Display the resulting frame
	cv2.imshow('frame', crop_img)
	cv2.waitKey(1)

def parallel(data):
	#Parallels cones
	#Define lists for left and right side of lidar/car	
	laser_right_temp = []
	laser_right = []
	laser_left = []
	
	#Append data from lidar to lists
	laser_right_temp += data.ranges[20:540]
	laser_left += data.ranges[541:1060]
	
	#Reverse list so that right side is measured from center
	laser_right += laser_right_temp[::-1]

	#DEBUG (DELETE WHEN DONE)
	print("Lowest Right value: ", np.amin(laser_right) , "at ", (laser_right.index(np.amin(laser_right))/4), "degrees off center")
	print("Lowest Left value:  ", np.amin(laser_left) , "at ", (laser_left.index(np.amin(laser_left))/4), "degrees off center")
	
	closest_dist = np.amin(laser_right)
	closest_angle = laser_right.index(np.amin(laser_right))/4
	angle_offset = 90 - closest_angle

	#Find next closest object on left side
	laser_second = []
	laser_second += laser_right[0:(closest_angle)]
	print("Second value:  ", np.amin(laser_second) , "at ", (laser_second.index(np.amin(laser_second))/4), "degrees off center")
	second_dist = np.amin(laser_second)
	second_angle = laser_second.index(np.amin(laser_second))/4
	second_offset = 90 - second_angle
	
	#Closest Position
	y = closest_dist * math.sin(math.radians(angle_offset/4))
	temp_x = closest_dist * math.cos(math.radians(angle_offset/4))
	x = temp_x - 0.635
	print("Y ", y)
	print("X ", x)

	#Next position
	y_next = second_dist * math.sin(math.radians(second_offset/4))
	x_next_temp = second_dist * math.cos(math.radians(second_offset/4))
	x_next = x_next_temp - 0.635
	print("Y Next ", y_next)
	print("X Next ", x_next)
	
	if(closest_angle >= 90):
		drive(0.5, (x_next) * -1)
	else:
		drive(0.5, (x * -1))

def timer():
	temp = time.time()
	return temp

def weave(data):
	global substate, time_start, minTime, orbit_dist
	print("SUBSTATE: ", substate)
	print("TIME: ", time_start)
	temp = timer()
	print("NEWTIME: ", temp)
	current_time = timer() - time_start
	print("CURRENT TIME", current_time)
	if(current_time >= minTime):
		print("ABLE TO SWITCH SUBSTATE")
	#Weave thru cones
	#Define lists for left and right side of lidar/car	
	laser_right_temp = []
	laser_right = []
	laser_left = []
	
	#Append data from lidar to lists
	laser_right_temp += data.ranges[20:820] # 5 degrees to 205 degrees
	laser_left += data.ranges[260:1060] # 65 degrees to 265 degrees
	
	#Reverse list so that right side is measured from center
	laser_right += laser_right_temp[::-1]

	if(substate == 1):
		#weave on left side
		closest_dist = np.amin(laser_right)
		closest_angle = laser_right.index(np.amin(laser_right))/4
		angle_offset = 140 - closest_angle
	
		y = closest_dist * math.sin(math.radians(angle_offset/4))
		temp_x = closest_dist * math.cos(math.radians(angle_offset/4))
		x = temp_x - 0.5

		print("Distance: ", closest_dist)
		print("ANGLE: ", closest_angle)
		print("Y ", y)
		print("X ", x)
		print("TIME: ", time.time())

		#check for new objects on left side
		#closest_dist_left = np.amin(laser_left)
		#if(closest_angle >=140 and closest_dist_left <= orbit_dist):
		#	current_time = timer() - time_start
		#	print("CURRENT TIME", current_time)
		#	if(current_time >= minTime):
		#		substate = 2
		#		time_start = timer()
				
		
		#orbit cone on right
		if(closest_dist <= orbit_dist):	#if object is close enough drive/ else do nothing
			if(closest_angle <= 100):	#if cone is on wrong side
				drive(0.5, 1)
			else:
				if(y <= 0):	#if object is beyond 90 degrees on right side
					drive(0.5, -1)
				else:			#drive normally
					drive(0.5, (x * -1))		

	if(substate == 2):
		#weave on right side
		closest_dist = np.amin(laser_left)
		closest_angle = laser_left.index(np.amin(laser_left))/4
		angle_offset = 140 - closest_angle
	
		y = closest_dist * math.sin(math.radians(angle_offset/4))
		temp_x = closest_dist * math.cos(math.radians(angle_offset/4))
		x = temp_x - 0.5

		print("Distance: ", closest_dist)
		print("ANGLE: ", closest_angle)
		print("Y ", y)
		print("X ", x)
		print("TIME: ", time.time())

		#check for new objects on right side
		#closest_dist_right = np.amin(laser_right)
		#if(closest_angle >= 140 and closest_dist_right <= orbit_dist):
		#	current_time = timer() - time_start
		#	print("CURRENT TIME", current_time)
		#	if(current_time >= minTime):
		#		substate = 1
		#		time_start = timer()			

		#orbit cone on left
		if(closest_dist <= orbit_dist):	#if object is close enough drive/ else do nothing
			if(closest_angle <= 100):
				drive(0.5, -1)
			else:
				if(y <= 0):	#if object is beyond 90 degrees on left side
					drive(0.5, 1)
				else:			#drive normally
					drive(0.5, x)

def lidar(data):
	#Callback for lidar

	#Full stop
	#Define list for center of car
	center = []
	#Append data from lidar to list
	center += data.ranges[420:660]	#105 degrees to 165 degrees
	#Find closest object
	center_closest_dist = np.amin(center)
	#Check if closest object is too close
	if(center_closest_dist <= full_stop):
		drive(0,0)
	else:
	#Drive normally if no objects too close
		if(state == 1):
			parallel(data)	
		if(state == 2):
			weave(data)
	
def listener():
	rp.init_node("Challenge3", anonymous = True)
	rp.Subscriber("zedRight", Image, zedCam)
	rp.Subscriber("/scan", LaserScan, lidar)
	rp.spin()
	
if __name__ =="__main__":
	try:
		listener()
	except rp.ROSInterruptException:
		pass
