import rospy as rp
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

pub = rp.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size = 10)
steeringErrors = [0]

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

def getOutput(error):
	global steeringErrors

	print ("ERROR", error)
	# PID Constants
	kP = 0.7		#0.2
	kI = 0.0	#0.000000001
	kD = 0		#2.5


	# Calculate new PID #PleaseWork!!!
	while(True):
		integral = sum(steeringErrors)
		#error = 145 - angle
		integral = integral + error
		if(error == 0):
			integral = 0
		if(abs(error) > 40):
			integral = 0
		derivative = error - steeringErrors[-2]
		steeringErrors += [error]
		turn = (kP * error) + (kI * integral) + (kD * derivative)
		# Result can only be between -1 and 1
		if(turn > 1):
			turn = 1
		elif(turn < -1):
			turn = -1
		return turn


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

	#Determine speed
	#If center distance is greater than 0.4 meters
	if(data.ranges[540]>0.4):
		#speed is absolute value of 1 minus turning angle
		speed = abs(1 - result)
		#if speed is less than 0.3 set speed to 0.3 as hard turns will set speed to 0.0
		if(speed <0.3):
			speed = 0.3
	else:
		speed = data.ranges[540] - 0.2
	# Send Message with speed and angle
	drive(speed,result)

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
