import rospy as rp
from sensor_msgs.msg import LaserScan

def callback(data):
	print ("CENTER", data.ranges[540])
	print ("Left Angle", data.ranges[780])
	print ("Right Angle", data.ranges[300])
	print ("Hard Left", data.ranges[940]
	print ("Hard Right", data.ranges[140])

	print ("Offset ", data.ranges[940] -  data.ranges[140])

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
