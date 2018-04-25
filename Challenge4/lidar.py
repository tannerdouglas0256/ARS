import rospy as rp
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

pub = rp.Publisher("Lidar", String, queue_size = 10)
fullstop = 0.2
min_gap = 0.0
angle_offset = 140
x_offset = 0.1

def callback(data):
	global steeringErrors, fullstop, min_gap, angle_offset
	
	#Define lists for left and right side of lidar/car	
	laser_right_temp = []
	laser_right = []
	laser_left = []
	
	#Append data from lidar to lists
	laser_right_temp += data.ranges[180:540] # 90 degrees to 135 degrees
	laser_left += data.ranges[540:900] # 135 degrees to 225 degrees
	
	#Reverse list so that right side is measured from center
	laser_right += laser_right_temp[::-1]

	#Full stop
	#Define list for center of car
	center = []
	#Append data from lidar to list
	center += data.ranges[420:660]	#105 degrees to 165 degrees
	#Find closest object
	center_closest_dist = np.amin(center)
	#Check if closest object is too close
	if(center_closest_dist <= fullstop):
		error = "stop"
		print ("STOP", fullstop)
		print("ERROR: ", center_closest_dist)
		print("ERROR: ", center.index(np.amin(center)))
		pub.publish(str(error))		

	else:
		#Drive normally if no objects too close
		
		#Find distance on right
		closest_dist_right = np.amin(laser_right)
		closest_angle_right = laser_right.index(np.amin(laser_right))/4
		angle_offset_right = angle_offset - closest_angle_right
	
		y_R = closest_dist_right * math.sin(math.radians(angle_offset_right/4))
		temp_x_R = closest_dist_right * math.cos(math.radians(angle_offset_right/4))
		x_R = temp_x_R - x_offset

		#Find distance on left
		closest_dist_left = np.amin(laser_left)
		closest_angle_left = laser_left.index(np.amin(laser_left))/4
		angle_offset_left = angle_offset - closest_angle_left
	
		y_L = closest_dist_left * math.sin(math.radians(angle_offset_left/4))
		temp_x_L = closest_dist_left * math.cos(math.radians(angle_offset_left/4))
		x_L = temp_x_L - x_offset

		#equation should be abs(x_r - x_L) since x_L is suppossed to be negative and can never move
		#from the left side of the car. Because all lidar data is positive instead of making x_L
		#negative we leave it positive and add instead of subtract
		delta_x = x_R + x_L
		if(delta_x >= min_gap):
			#if gap is big enough drive thru it
			if(x_L < x_R):
				#steer to right of object
				#if(y_R <= 0):	#if object is beyond 90 degrees on right side
				#	error = -1
				#else:			#drive normally
				error = (x_R * -1)
			elif(x_L > x_R):
				#steer to left of object
				#if(y_L <= 0):	#if object is beyond 90 degrees on left side
				#	error = 1
				#else:			#drive normally
				error = (x_L * 1)
			else:
				#objects are equidistant from each other... move along, move along....
				error = 0

			#result = getOutput(error)
			#print("Steering: " + str(result))
			print("ERROR: ", error)
			pub.publish(str(error))
		else:
			print("DeltaX not working")		



def listener():
	rp.init_node("laser_listener", anonymous = True)
	rp.Subscriber("/scan", LaserScan, callback)
	rp.spin()
	
if __name__ =="__main__":
	try:
		listener()
	except rp.ROSInterruptException:
		pass
