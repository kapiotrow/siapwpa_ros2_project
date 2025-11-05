#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# Initialize the CvBridge class
bridge = CvBridge()

def image_callback(msg):
    # Convert ROS Image message to OpenCV image
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8').astype('uint8')
    except Exception as e:
        rospy.logerr("Error converting ROS Image to OpenCV: %s", e)
        return

    # Display the image
    cv2.imshow("ROS Image", cv_image)
    cv2.waitKey(1)  # Required to update the OpenCV window

def main():
    rospy.init_node('image_listener', anonymous=True)
    
    # Subscribe to your topic
    rospy.Subscriber("/world/macnum_drive/model/samochod/link/base_footprint/sensor/realsens_rgbd/image", Image, image_callback)
    
    rospy.loginfo("Subscribed to image topic. Waiting for images...")
    
    rospy.spin()  # Keep the node running
    
    # Destroy OpenCV windows when done
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

