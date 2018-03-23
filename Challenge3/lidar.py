import rospy as rp
import numpy as np
from sensor_msgs.msg import LaserScan

def callback(data):

	#print ("", data)
	#print ("CENTER", data.ranges[540])
	#print ("Left Angle", data.ranges[780])
	#print ("Right Angle", data.ranges[300])
	#print ("Hard Left", data.ranges[940])
	#print ("Hard Right", data.ranges[140])

	#print ("Offset ", data.ranges[780] - data.ranges[300])

	#Calculate how for of center we are (meters)	
	#error = data.ranges[780] - data.ranges[300]
	
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

	


def listener():
	rp.init_node("laser_listener", anonymous = True)
	rp.Subscriber("/scan", LaserScan, callback)
	#rate = rp.Rate(60)
	rp.spin()
	
if __name__ =="__main__":
	try:
		listener()
	except rp.ROSInterruptException:
		pass
