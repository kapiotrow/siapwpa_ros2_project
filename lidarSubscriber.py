import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        # Subskrybujemy topic, na którym przesyłane są dane LIDAR-a
        self.subscription = self.create_subscription(
            LaserScan,
            '/world/mecanum_drive/model/samochod/link/base_footprint/sensor/velodyne_lidar/scan',
            self.listener_callback,
            10  # Kolejka 10 wiadomości
        )
        self.subscription  # Aby uniknąć ostrzeżenia o nieużywanym obiekcie

    def listener_callback(self, msg):
        # Wyświetlamy część danych z LaserScan
        self.get_logger().info(f"Received LIDAR data:")
        self.get_logger().info(f"Angle min: {msg.angle_min}")
        self.get_logger().info(f"Angle max: {msg.angle_max}")
        self.get_logger().info(f"Number of ranges: {len(msg.ranges)}")
        self.get_logger().info(f"Ranges (first 10 points): {msg.ranges[:10]}")

        # Dodatkowo, możemy wyświetlić kilka odległości, aby zobaczyć pierwsze pomiary
        for i in range(min(10, len(msg.ranges))):  # Wyświetlamy tylko pierwsze 10 odległości
            self.get_logger().info(f"Angle: {msg.angle_min + i * msg.angle_increment} | Range: {msg.ranges[i]}")


def main(args=None):
    rclpy.init(args=args)

    lidar_subscriber = LidarSubscriber()

    # Uruchamiamy node
    rclpy.spin(lidar_subscriber)

    # Zamykanie aplikacji, jeśli node jest zatrzymany
    lidar_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
