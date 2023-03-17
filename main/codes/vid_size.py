import rospy
from sensor_msgs.msg import Image

# Define the ROS image topic to subscribe to
image_topic = "/camera/color/image_raw"

# Define the callback function to process the ROS messages
def callback(image_msg):
    # Get the height and width of the image
    height = image_msg.height
    width = image_msg.width

    print("Video streaming size: {}x{} pixels".format(width, height))

# Subscribe to the image topic
rospy.init_node("image_subscriber")
rospy.Subscriber(image_topic, Image, callback)

# Spin the node
rospy.spin()
