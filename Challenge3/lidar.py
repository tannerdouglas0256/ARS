import cv2
import rospy as rp
import numpy as np
import math
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
bridge = CvBridge()

pub = rp.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size = 10)
image_right = []

#How close can an object be infront of us before we stop
full_stop = 0.3
#Determines what portion of the course we are in
# 0 = Initial Alignment
# 1 = Parallel
# 2 = Weave
state = 1
#Remember what side we are weaving on
# 1 = Right
# 2 = Left
substate = 1

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
	image_right = bridge.imgmsg_to_cv2(data, desired_encoding = "passthrough")
	cv2.imshow("", image_right)
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
	
	#Position on LEFT
	y = closest_dist * math.sin(math.radians(angle_offset/4))
	temp_x = closest_dist * math.cos(math.radians(angle_offset/4))
	x = temp_x - 0.635
	print("Y ", y)
	print("X ", x)

	#Next position on left
	y_next = second_dist * math.sin(math.radians(second_offset/4))
	x_next_temp = second_dist * math.cos(math.radians(second_offset/4))
	x_next = x_next_temp - 0.635
	print("Y Next ", y_next)
	print("X Next ", x_next)


#	if(left_y <= 0):	#if object is beyond 90 degrees on left side
#		drive(0.5, 1)
#	else:			#drive normally
#		drive(0.5, left_x)
	
	#Find difference in x distances of both objects
	#new_x = left_x + left_x_next
	#drive(0.5, new_x)

	if(closest_angle >= 90):
		drive(0.5, x_next)
	else:
		drive(0.5, x)


def weave(data):
	#Weave thru cones
	#Define lists for left and right side of lidar/car	
	laser_right_temp = []
	laser_right = []
	laser_left = []
	
	#Append data from lidar to lists
	laser_right_temp += data.ranges[20:540]
	laser_left += data.ranges[541:1060]
	
	

	if(substate == 1):
		#weave on left side
		x = 1
	if(substate == 2):
		#weave on right side
		x = 1

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
