import rclpy
from rclpy.node import Node
import cv2

from sensor_msgs.msg import Image 


class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        cv2.imshow("image from camera", msg.data)
        # TODO: 


def main(args=None):
    rclpy.init(args=args)

    camera_subscriber = CameraSubscriber()

    rclpy.spin(camera_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cv2.destroyAllWindows()
    camera_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()