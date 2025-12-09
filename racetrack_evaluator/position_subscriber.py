#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
import numpy as np

POSE_TOPIC = '/world/mecanum_drive/pose/info'
ROBOT_MODEL_NAME = 'samochod'  # model, którego pozycję chcemy odczytać

class PoseInfoSubscriber(Node):

    def __init__(self):
        super().__init__('pose_info_subscriber')
        self.subscription = self.create_subscription(
            PoseArray,
            POSE_TOPIC,
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.position = None
        self.orientation = None

    def listener_callback(self, msg: PoseArray):
        # Zakładamy, że pierwszy Pose w tablicy odpowiada naszemu samochodowi
        if len(msg.poses) == 0:
            self.get_logger().warn("Brak danych w PoseArray")
            return

        pose: Pose = msg.poses[2]  # lub wyszukać po indeksie/ nazwie modelu
        self.position = np.array([pose.position.x,
                                  pose.position.y,
                                  pose.position.z])
        self.orientation = np.array([pose.orientation.x,
                                     pose.orientation.y,
                                     pose.orientation.z,
                                     pose.orientation.w])

        self.get_logger().info(f"Pozycja: {self.position}, Orientacja: {self.orientation}")


def main(args=None):
    rclpy.init(args=args)
    pose_subscriber = PoseInfoSubscriber()
    rclpy.spin(pose_subscriber)
    pose_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
