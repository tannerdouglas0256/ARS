import rospy as rp
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

pub = rp.Publisher("Lidar", String, queue_size = 10)

def callback(data):
	global steeringErrors

	print ("CENTER", data.ranges[540])
	print ("Left Angle", data.ranges[780])
	print ("Right Angle", data.ranges[300])
	print ("Hard Left", data.ranges[940])
	print ("Hard Right", data.ranges[140])

	print ("Offset ", data.ranges[780] - data.ranges[300])

	#Calculate how for of center we are (meters)	
	error = data.ranges[780] - data.ranges[300]

	# Add to error array
	steeringErrors += [error]
	
	# Only store previous 100 errors
	if(len(steeringErrors)>100):
		steeringErrors = steeringErrors[1:]
	
	# Send Error to PID
	result = getOutput(error)
	print("Steering: " + str(result))
	pub.publish(str(result))

def listener():
	rp.init_node("laser_listener", anonymous = True)
	rp.Subscriber("/scan", LaserScan, callback)
	rp.spin()
	
if __name__ =="__main__":
	try:
		listener()
	except rp.ROSInterruptException:
		pass
