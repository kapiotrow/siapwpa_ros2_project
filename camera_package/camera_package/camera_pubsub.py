import rclpy
from rclpy.node import Node
from camera_subscriber import RGBCameraSubscriber
import cv2
from cv_bridge import CvBridge
from line_follower import LineFollower
from sensor_msgs.msg import Image 
from geometry_msgs.msg import Twist

VEL_TOPIC = '/model/samochod/cmd_vel'
TOPIC_RGB = '/world/mecanum_drive/model/samochod/link/base_footprint/sensor/realsense_rgbd/image'
TOPIC_DEPTH = '/world/mecanum_drive/model/samochod/link/base_footprint/sensor/realsense_depth/depth_image'
bridge = CvBridge()


class VelocityPublisher(Node):

    def __init__(self):
        super().__init__('velocity_publisher')

        self.publisher_ = self.create_publisher(Twist, VEL_TOPIC, 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

        # SUBSKRYBER W TYM SAMYM NODE
        self.subscription = self.create_subscription(
            Image,
            TOPIC_RGB,
            self.camera_callback,
            10
        )

        self.line_follower = LineFollower()
        self.directions_vector = [0.0, 0.0]

    def camera_callback(self, msg):
        cv_image = bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.directions_vector = self.line_follower.pid_control(cv_image)

    def timer_callback(self):

        visibility = self.directions_vector[0]
        offset = self.directions_vector[1]

        max_speed_forward = 1.0  # m/s
        max_speed_sideways = 4.0 # m/s

        msg = Twist()
        msg.linear.x = max_speed_sideways * offset   # rotate-right (positive), rotate-left (negative)
        msg.linear.y = 0.0 # forward (positive), backward (negative)
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = -max_speed_forward * visibility

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

  
if __name__ == '__main__':
    main()
