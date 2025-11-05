import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image 

bridge = CvBridge()


TOPIC_RGB = '/world/mecanum_drive/model/samochod/link/base_footprint/sensor/realsense_rgbd/image'
TOPIC_DEPTH = '/world/mecanum_drive/model/samochod/link/base_footprint/sensor/realsense_depth/depth_image'


class RGBCameraSubscriber(Node):
    
    def __init__(self):
        super().__init__('rgb_camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            TOPIC_RGB,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8').astype('uint8')
        except Exception as e:
            print("Error converting ROS Image to OpenCV: %s", e)

        cv2.imshow("RGB image from camera", cv_image)
        cv2.waitKey(1)  # Required to update the OpenCV window

class DepthCameraSubscriber(Node):
    
    def __init__(self):
        super().__init__('depth_camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            TOPIC_DEPTH,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        cv2.imshow("Depth image from camera", msg.data)
        cv2.waitKey(1)  # Required to update the OpenCV window



def main(args=None):
    rclpy.init(args=args)

    rgb_camera_subscriber = RGBCameraSubscriber()

    rclpy.spin(rgb_camera_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cv2.destroyAllWindows()
    rgb_camera_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()