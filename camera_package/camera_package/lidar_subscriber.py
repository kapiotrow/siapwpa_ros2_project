import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')

        self.subscription = self.create_subscription(
            LaserScan,
            '/world/mecanum_drive/model/samochod/link/base_footprint/sensor/velodyne_lidar/scan',
            self.listener_callback,
            10
        )
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(f"Received LIDAR data:")
        self.get_logger().info(f"Angle min: {msg.angle_min}")
        self.get_logger().info(f"Angle max: {msg.angle_max}")
        self.get_logger().info(f"Number of ranges: {len(msg.ranges)}")
        # self.get_logger().info(f"Ranges (first 10 points): {msg.ranges[:10]}")


def main(args=None):
    rclpy.init(args=args)

    lidar_subscriber = LidarSubscriber()

    rclpy.spin(lidar_subscriber)

    lidar_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
