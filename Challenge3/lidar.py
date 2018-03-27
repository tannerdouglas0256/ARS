import rospy as rp
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def callback(data):

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
	
	#Position on LEFT
	left_y = left_closest_dist * math.sin(math.radians(left_angle_offset))
	temp_left_x = left_closest_dist * math.cos(math.radians(left_angle_offset))
	left_x = temp_left_x - 0.635
	print("LEFT Y ", left_y)
	print("LEFT X ", left_x)

	data = [left_y, left_x]
	
	pub.publish(data)
	
	

def listener():
	rp.init_node("laser_listener", anonymous = True)
	rp.Subscriber("/scan", LaserScan, callback)
	rp.spin()
	
if __name__ =="__main__":
	try:
		listener()
	except rp.ROSInterruptException:
		pass
