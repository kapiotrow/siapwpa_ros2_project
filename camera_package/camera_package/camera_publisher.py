import rclpy
from rclpy.node import Node
from camera_subscriber import RGBCameraSubscriber
import cv2
import numpy as np

from geometry_msgs.msg import Twist
VEL_TOPIC = '/model/samochod/cmd_vel'


class velocityPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, VEL_TOPIC, 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.subscriber = RGBCameraSubscriber()

        rclpy.spin(self.subscriber)
        print(1)

    def timer_callback(self):

        directions = self.subscriber.directions_vector
        visibility = directions[0]
        offset = directions[1]

        max_speed_forward = 3.0  # m/s
        max_speed_sideways = 1.5  # m/s
        
        msg = Twist()
        msg.linear.x = max_speed_sideways * offset  # rotate-right (positive), rotate-left (negative)
        msg.linear.y = max_speed_forward * visibility  # forward (positive), backward (negative)
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing sideways velocity: {msg.linear.x} m/s')
        self.get_logger().info(f'Publishing forward velocity: {msg.linear.y} m/s')
        self.get_logger().info(f'Publishing upward velocity: {msg.linear.z} m/s')

        # python3 camera_package/camera_package/camera-publisher.py 

        


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = velocityPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
