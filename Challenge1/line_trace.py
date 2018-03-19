import cv2
import numpy as np
import rospy as rp
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
pub = rp.Publisher('CVOutput', String, queue_size = 10)
 
def callback(data):
	cv_img = bridge.imgmsg_to_cv2(data, desired_encoding = "passthrough")

	# Crop the image
	crop_img = cv_img[250:675, 50:700]
	
	# Uncomment if you want to use original image
	#crop_img = cv_img

	kernel = np.ones((5,5),np.uint8)
	# Gaussian Blur image
	crop_img = cv2.GaussianBlur(crop_img, (11,11), 0)

	# Convert RGB image to HSV image
	hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

	# Grayscale image
	gray_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)

	# Define range of blue color in HSV
	lower_blue = np.array([100, 50, 50], dtype = "uint8")
	upper_blue = np.array([120, 255, 255], dtype = "uint8")

	# Mask of blue in the image
	blue_mask = cv2.inRange(hsv_img, lower_blue, upper_blue)
	blue_mask = cv2.erode(blue_mask, kernel, iterations = 2)
	blue_mask = cv2.dilate(blue_mask, kernel, iterations = 2)
	blue_mask = cv2.dilate(blue_mask, kernel, iterations = 2)
	blue_mask = cv2.erode(blue_mask, kernel, iterations = 2)

	# Display the resulting frame
	cv2.imshow('mask', blue_mask)
	cv2.waitKey(1)

	# Color thresholding
	ret,thresh = cv2.threshold(blue_mask, 80, 255, cv2.THRESH_BINARY)

	# Find the contours of the frame
	_,contours,hierarchy = cv2.findContours(thresh.copy(), 1, cv2.CHAIN_APPROX_NONE)

	# Find the biggest contour
	if len(contours) > 0:
		c = max(contours, key = cv2.contourArea)
		M = cv2.moments(c)

	cx = int(M['m10']/M['m00'])
	cy = int(M['m01']/M['m00'])

	cv2.line(crop_img, (cx,0), (cx,720), (255,0,0), 1)
	cv2.line(crop_img, (0,cy), (1280,cy), (255,0,0), 1)
	cv2.drawContours(crop_img, contours, -1, (0,0,255), 1)

	print(cx)
	pub.publish(str(cx))

	# Display the resulting frame
	cv2.imshow('frame', crop_img)
	cv2.waitKey(1)

def listener():
	rp.init_node("OpenCV", anonymous = True)
	rp.Subscriber('zedRight', Image, callback)
	rate = rp.Rate(60)
	rp.spin()

if __name__ == "__main__":
	try:
		listener()
	except rp.ROSInterruptException:
		pass
