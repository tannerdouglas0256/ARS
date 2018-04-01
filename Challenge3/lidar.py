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

#Determines what portion of the course we are in
# 0 = Initial Alignment
# 1 = Parallel
# 2 = Weave
state = 0

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
	
	left_closest_dist = np.amin(laser_left)
	left_closest_angle = laser_left.index(np.amin(laser_left))/4
	left_angle_offset = 90 - left_closest_angle

	#Find next closest object on left side
	laser_left_second = []
	laser_left_second += laser_left[0:(left_closest_angle)]
	print("Second Left value:  ", np.amin(laser_left_second) , "at ", (laser_left_second.index(np.amin(laser_left_second))/4), "degrees off center")
	left_second_dist = np.amin(laser_left_second)
	left_second_angle = laser_left_second.index(np.amin(laser_left_second))/4
	left_second_offset = 90 - left_second_angle
	
	#Position on LEFT
	left_y = left_closest_dist * math.sin(math.radians(left_angle_offset/4))
	temp_left_x = left_closest_dist * math.cos(math.radians(left_angle_offset/4))
	left_x = temp_left_x - 0.635
	print("LEFT Y ", left_y)
	print("LEFT X ", left_x)

	#Next position on left
	left_y_next = left_second_dist * math.sin(math.radians(left_second_offset/4))
	left_x_next_temp = left_second_dist * math.cos(math.radians(left_second_offset/4))
	left_x_next = left_x_next_temp - 0.635
	print("Left Y Next ", left_y_next)
	print("Left X Next ", left_x_next)


	if(left_y <= 0):	#if object is beyond 90 degrees on left side
		drive(0.5, 1)
	else:			#drive normally
		drive(0.5, left_x)
	
	#Find difference in x distances of both objects
	new_x = left_x + left_x_next

	drive(0.5, new_x)


def weave(lidar):
	#Weave thru cones
	x =1
def lidar(data):

	parallel(data)	
	
#	if(left_y == 0):
#		if(left_x == 0):
#			#go to next operation
#			state = 1
#		if(left_x != 0):
#			#GO BACK AND RE-ATTEMPT
#			drive(-2, 0)
#	if(left_y > 0):
#		#DRIVE FORWARD
#		drive(left_y, left_x)
	

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
