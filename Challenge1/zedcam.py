import cv2
import numpy as np
import rospy as rp
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def poly_talker():
	pub = rp.Publisher('zedRight', Image, queue_size = 1)
	rp.init_node('ZEDcam', anonymous = True)
	rate = rp.Rate(60)
	cap = cv2.VideoCapture(1)
	
	while not rp.is_shutdown():
		print("PUBLISHING...")
		ret,frame = cap.read()
		rightEye = frame[0:376, 672:1344]
		output = bridge.cv2_to_imgmsg(rightEye, encoding = "passthrough")
		pub.publish(output)
		rate.sleep()

if __name__ == "__main__":
	try:
		poly_talker()
	except rp.ROSInterruptException:
		pass

