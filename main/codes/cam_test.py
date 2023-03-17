import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Initialize ROS node
rospy.init_node('video_capture')

# Create a bridge between ROS and OpenCV
bridge = CvBridge()

# Define a callback function to process the images
def process_image(image):
    # Convert the ROS image message to an OpenCV image
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')

    # Show the image in a window
    cv2.imshow('Video', cv_image)

    # Exit if the user presses the 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        rospy.signal_shutdown('User requested shutdown')

# Subscribe to the camera rostopic
rospy.Subscriber('/camera/color/image_raw', Image, process_image)

# Spin the ROS node to receive messages
rospy.spin()

# Close the OpenCV window
cv2.destroyAllWindows()
