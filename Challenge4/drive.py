import rospy as rp
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import String

pub = rp.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size = 10)
steeringErrorsZed = [0]
steeringErrorsLidar = [0]
lidarSteering = 0
zed_result = 0
angle = 0
red = 0
green = 0

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

def getOutputZed(error):
	global steeringErrorsZed

	# PID Constants
	kP = 0.002
	kI = 0.0
	kD = 0.0012


	# Calculate new PID #PleaseWork!!!
	while(True):
		integral = sum(steeringErrorsZed)
		#error = 145 - angle
		integral = integral + error
		if(error == 0):
			integral = 0
		if(abs(error) > 40):
			integral = 0
		derivative = error - steeringErrorsZed[-2]
		steeringErrorsZed += [error]
		speed = (kP * error) + (kI * integral) - (kD * derivative)
		# Result can only be between -1 and 1
		if(speed > 1):
			speed = 1
		elif(speed < -1):
			speed = -1
		return speed

def getOutputLidar(error):
	global steeringErrorsLidar

	print ("ERROR", error)
	# PID Constants
	kP = 0.4
	kI = 0.0
	kD = 0.15

	# Calculate new PID #PleaseWork!!!
	while(True):
		integral = sum(steeringErrorsLidar)
		#error = 145 - angle
		integral = integral + error
		if(error == 0):
			integral = 0
		if(abs(error) > 40):
			integral = 0
		#derivative = error - steeringErrorsLidar[-2]
		steeringErrorsLidar += [error]
		turn = (kP * error) + (kI * integral)# + (kD * derivative)
		# Result can only be between -1 and 1
		if(turn > 1):
			turn = 1
		elif(turn < -1):
			turn = -1
		return turn

def shortcut(data):
	global steeringErrorsZed, green
	green = float(data.data)
	error = 290 - green

	# Add to error array
	steeringErrorsZed += [error]
	
	# Only store previous 100 errors
	if(len(steeringErrorsZed)>100):
		steeringErrorsZed = steeringErrorsZed[1:]
	
	# Send Error to PID
	global zed_result
	zed_result = getOutputZed(error)
	print("Angle: " + str(angle))
	print("Error: " + str(error))
	print("Steering: " + str(getOutputZed(error)))

def stop(data):
	global steeringErrorsZed, red
	red = float(data.data)
	error = 290 - red

	# Add to error array
	steeringErrorsZed += [error]
	
	# Only store previous 100 errors
	if(len(steeringErrorsZed)>100):
		steeringErrorsZed = steeringErrorsZed[1:]
	
	# Send Error to PID
	global zed_result
	zed_result = getOutputZed(error)
	print("Angle: " + str(angle))
	print("Error: " + str(error))
	print("Steering: " + str(getOutputZed(error)))

def followLine(data):
	global steeringErrorsZed, angle
	angle = float(data.data)
	error = 290 - angle

	# Add to error array
	steeringErrorsZed += [error]
	
	# Only store previous 100 errors
	if(len(steeringErrorsZed)>100):
		steeringErrorsZed = steeringErrorsZed[1:]
	
	# Send Error to PID
	global zed_result
	zed_result = getOutputZed(error)
	print("Angle: " + str(angle))
	print("Error: " + str(error))
	print("Steering: " + str(getOutputZed(error)))

def followWall(data):
	speed = 0.5
	global lidarSteering, zed_result
	
	#print("STEERING: ", steering)
	if(data.data == "stop"):
		print("STOP")
	else:
		lidarSteering = float(data.data)
		if(red):
			print("END OF COURSE")
		elif(green):
			print("SHORTCUT")
			drive(0.4, zed_result)
		elif(angle):
			print("ZEDSTEERING")
			drive(0.4, zed_result)
		else:
			print("LIDARSTEERING: ", lidarSteering)
			lidarOutput = getOutputLidar(lidarSteering)
			drive(0.5, lidarOutput)	

def listener():
	rp.init_node("VESC_Steering", anonymous = False)
	rp.Subscriber("CVOutput", String, followLine)
	rp.Subscriber("shortcut", String, shortcut)	
	rp.Subscriber("stop", String, stop)
	rp.Subscriber("Lidar", String, followWall)
	rate = rp.Rate(60)
	rp.spin()
	
if __name__ =="__main__":
	try:
		listener()
	except rp.ROSInterruptException:
		pass
