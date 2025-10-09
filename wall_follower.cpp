#!/usr/bin/env python3

# Released under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
# Author: Claude Sammut
# Last Modified: 2024.10.14

# ROS 2 program to subscribe to real-time streaming 
# video from TurtleBot3 Pi camera and find coloured landmarks.

# Colour segementation and connected components example added by Claude sSammut
	
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
import cv2 # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import math

from sensor_msgs.msg import Image # Image is the message type
from sensor_msgs.msg import LaserScan # Laser scan message type
from sensor_msgs.msg import CameraInfo # Need to know camera frame
from geometry_msgs.msg import Point, PointStamped

import wall_follower.landmark
from wall_follower.landmark import marker_type


field_of_view_h = 62.2
field_of_view_v = 48.8


class SeeMarker(Node):
	"""
	Create an ImageSubscriber class, which is a subclass of the Node class.
	"""
	def __init__(self):
		"""
		Class constructor to set up the node
		"""
		# Initiate the Node class's constructor and give it a name
		super().__init__('see_marker')

		self.range = 0
		self.prev_h = 0
		self.prev_r = 0
			
		# Create the subscriber. This subscriber will receive an Image
		# from the video_frames topic. The queue size is 10 messages.
		self.subscription = self.create_subscription(
			Image,
			'/camera/image_raw', 
			self.listener_callback, 
			10)
		self.subscription # prevent unused variable warning
			
		# Used to convert between ROS and OpenCV images
		self.br = CvBridge()

		self.point_publisher = self.create_publisher(PointStamped, '/marker_position', 10)


	def listener_callback(self, data):
		"""
		Callback function.
		"""
		# Display the message on the console
		# self.get_logger().info('Receiving video frame')
 
		# Convert ROS Image message to OpenCV image
		current_frame = self.br.imgmsg_to_cv2(data, 'bgra8')

		# The following code is a simple example of colour segmentation
		# and connected components analysis
		
		# Convert BGR image to HSV
		hsv_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

		# Find pink blob
		pink_blob = segment(current_frame, hsv_frame, "pink")
		if pink_blob:
			(pink_x, pink_y, pink_h, p_d, p_a) = pink_blob

			for c in ["blue", "green", "yellow"]:
				blob = segment(current_frame, hsv_frame, c)
				if blob:
					(c_x, c_y, c_h, c_d, c_a) = blob

					# Check to see if the blobsa are verically aligned
					if abs(pink_x - c_x) > pink_h:
#						print(f'pink_x = {pink_x}, pink_y = {pink_y}, h = {pink_h}')
						continue

					marker_at = PointStamped()
					marker_at.header.stamp = self.get_clock().now().to_msg()
					marker_at.header.frame_id = 'camera_link'

					if c_y < pink_y:	# +y is down
#						print(c, "/ pink", f'{c_d:.2f}, {c_a:.2f}')
						marker_at.point.z = float(marker_type.index(c + '/pink'))
					else:
#						print("pink / ", c, f'{p_d:.2f}, {p_a:.2f}')
						marker_at.point.z = float(marker_type.index('pink/' + c))
					
					x, y = polar_to_cartesian(c_d, c_a)

					marker_at.point.x = x
					marker_at.point.y = y

#					print(f'Camera coordinates: {x}, {y}')
					self.point_publisher.publish(marker_at)
#					self.get_logger().info('Published Point: x=%f, y=%f, z=%f' %
#						(marker_at.point.x, marker_at.point.y, marker_at.point.z))


		# Display camera image
		cv2.imshow("camera", current_frame)	
		cv2.waitKey(1)


colours = {
	"pink":	 	((140,0,0), (170, 255, 255)),
	"blue":		((100,0,0), (130, 255, 255)),
	"green":	((40,0,0), (80, 255, 255)),
	"yellow":	((25,0,0), (32, 255, 255))
}


def segment(current_frame, hsv_frame, colour):
	"""
	Mask out everything except the specified colour
	Connect pixels to form a blob
	"""

	(lower, upper) = colours[colour]

	# Mask out everything except pink pixels
	mask = cv2.inRange(hsv_frame, lower, upper)
	result = cv2.bitwise_and(current_frame, current_frame, mask=mask)

	# Run 4-way connected components, with statistics
	blobs = cv2.connectedComponentsWithStats(mask, 4, cv2.CV_32S)

	# Display masked image
#	cv2.imshow("result", result)
 
	# Print statistics for each blob (connected component)
	return get_stats(blobs, colour)


def get_stats(blobs, colour):
	"""
	Print statistics for each blob (connected component)
	of the specified colour
	Return the centroid and height of the largest blob, if there is one.
	"""

	(numLabels, labels, stats, centroids) = blobs
	result = []
	
	if numLabels == 0:
		return None


	largest = 0
	rval = None
	centre = 320 # 640/2

	for i in range(1, numLabels):
		x = stats[i, cv2.CC_STAT_LEFT]
		y = stats[i, cv2.CC_STAT_TOP]
		w = stats[i, cv2.CC_STAT_WIDTH]
		h = stats[i, cv2.CC_STAT_HEIGHT]
		area = stats[i, cv2.CC_STAT_AREA]
		(cx, cy) = centroids[i]
#		print(colour, x, y, w, h, area, cx, cy)

		if area > largest:
			largest = area
			distance = 35.772 * pow(h, -0.859) # obtained experimentally
			aspect_ratio = h/w
			if aspect_ratio < 0.8:
				if cx < centre:
					cx += h-w
				else:
					cx -= h-w
			angle = (centre - cx) * field_of_view_h / 640
			if angle < 0:
				angle += 360
			rval = (cx, cy, h, distance, angle)

	return rval


def polar_to_cartesian(distance, angle):
	# Convert angle from degrees to radians
	angle_rad = math.radians(angle)

	# Calculate x and y coordinates
	x = distance * math.cos(angle_rad)
	y = distance * math.sin(angle_rad)

	return x, y


def main(args=None):
	
	# Initialize the rclpy library
	rclpy.init(args=args)
	
	# Create the node
	see_marker = SeeMarker()
	
	# Spin the node so the callback function is called.
	try:
		rclpy.spin(see_marker)
	except KeyboardInterrupt:
		exit()

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	see_marker.destroy_node()
	
	# Shutdown the ROS client library for Python
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()
