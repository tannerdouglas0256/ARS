import rospy as rp
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import String

pub = rp.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size = 10)
steeringErrorsZed = [0]
steeringErrorsLidar = [0]
lidarSteering

def drive(speed, steering):
	drive_msg_stamped = AckermannDriveStamped()
	drive_msg = AckermannDrive()
       	drive_msg.speed = 1.0
        drive_msg.steering_angle = steering
        drive_msg.acceleration = 0
        drive_msg.jerk = 0
        drive_msg.steering_angle_velocity = 0
	drive_msg_stamped.drive = drive_msg
	pub.publish(drive_msg_stamped)

def getOutputZed(error):
	global steeringErrorsZed

	# PID Constants
	kP = 0.7
	kI = 0.0
	kD = 0.0


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
		speed = (kP * error) + (kI * integral) + (kD * derivative)
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
def followLine(data):
	global steeringErrors
	speed = 0.8
	angle = float(data.data)
	error = 300 - angle

	# Add to error array
	steeringErrors += [error]
	
	# Only store previous 100 errors
	if(len(steeringErrors)>100):
		steeringErrors = steeringErrors[1:]
	
	# Send Error to PID
	result = getOutputZed(error)
	print("Angle: " + str(angle))
	print("Error: " + str(error))
	print("Steering: " + str(getOutput(error)))

	# Send Message with speed and angle
	drive(speed,result)

def followWall(data):
	speed = 0.5
	global lidarSteering = float(data.data)
	print("STEERING: ", steering)
	#drive(speed, steering)
	

def listener():
	rp.init_node("VESC_Steering", anonymous = False)
	rp.Subscriber("CVOutput", String, followLine)
	rp.Subscriber("Lidar", String, followWall)
	rate = rp.Rate(60)
	rp.spin()
	
if __name__ =="__main__":
	try:
		listener()
	except rp.ROSInterruptException:
		pass
