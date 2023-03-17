import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Create a CvBridge object to convert ROS messages to OpenCV images
bridge = CvBridge()

# Define the ROS image topic to subscribe to
image_topic = "/camera/color/image_raw"

# Define the callback function to process the ROS messages
def callback(image_msg):
    # Convert the ROS message to an OpenCV image
    image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")

    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Apply a Gaussian blur to the grayscale image
    blurred = cv2.GaussianBlur(gray, (11, 11), 0)

    # Threshold the blurred image to create a binary image
    thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]

    # Find contours in the binary image
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Loop over the contours to find the bullseye target
    for contour in contours:
        # Compute the area of the contour
        area = cv2.contourArea(contour)

        # If the area is too small or too large, skip this contour
        if area < 1000 or area > 5000:
            continue

        # Compute the centroid of the contour
        M = cv2.moments(contour)
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        # Draw a circle around the centroid
        cv2.circle(image, (cx, cy), 5, (0, 255, 0), -1)

        # Output the centroid coordinates
        print("Bullseye target centroid coordinates: ({}, {})".format(cx, cy))

    # Display the image with the bullseye target centroid marked
    cv2.imshow("Bullseye Target Detection", image)
    cv2.waitKey(1)

# Initialize the ROS node
rospy.init_node("bullseye_target_detection")

# Subscribe to the ROS image topic
rospy.Subscriber(image_topic, Image, callback)

# Spin the ROS node
rospy.spin()
