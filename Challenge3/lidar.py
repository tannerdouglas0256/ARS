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
		speed = 1 - abs(result)
		#if speed is less than 0.3 set speed to 0.3 as hard turns will set speed to 0.0
		if(speed <0.6):
			speed = 0.6
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
