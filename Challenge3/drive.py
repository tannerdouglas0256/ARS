import rospy as rp
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
	
	print("DATA: ", data)
	
#	if(left_y == 0):
#		if(left_x == 0):
#			#go to next operation
#		if(left_x != 0):
#			#GO BACK AND RE-ATTEMPT
#	if(left_y > 0):
#		#DRIVE FORWARD

def listener():
	rp.init_node("Driver", anonymous = True)
	rp.Subscriber(#"CHANGE THIS to our lidar function publisher", LaserScan, callback)
	#rate = rp.Rate(60)
	rp.spin()
	
if __name__ =="__main__":
	try:
		listener()
	except rp.ROSInterruptException:
		pass
