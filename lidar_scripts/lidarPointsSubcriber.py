#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import cv2
import os
from datetime import datetime

class LidarCV2Visualizer(Node):
    def __init__(self):
        super().__init__('lidar_cv2_visualizer')

        # Subskrypcja PointCloud2
        self.subscription = self.create_subscription(
            PointCloud2,
            '/world/mecanum_drive/model/samochod/link/base_footprint/sensor/velodyne_lidar/scan/points',
            self.callback,
            10
        )

        # Znamy dane z SDF → ustawiamy stałe:
        self.vertical_layers = 16         # z <vertical><samples>16</samples>
        self.horizontal_samples = 1800    # z <horizontal><samples>1800</samples>

        self.get_logger().info("OpenCV LiDAR visualizer uruchomiony.")

        log_dir = os.path.expanduser("lidar_logs")

        os.makedirs(log_dir, exist_ok=True)

        self.dist_log_path = os.path.join(log_dir, "lidar_distances.txt")

        self.dist_log_file = open(self.dist_log_path, "a", buffering=1)

        self.get_logger().info(f"LiDAR distances logged to {self.dist_log_path}")


    def callback(self, msg):
        # Pobranie punktów PointCloud2
        points_list = []

        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points_list.append([p[0], p[1], p[2]])

        points = np.array(points_list, dtype=np.float32)

        if points.shape[0] == 0:
            return

        # Obliczamy dystans r = sqrt(x²+y²+z²)
        distances = np.linalg.norm(points[:, :3], axis=1)

        timestamp = self.get_clock().now().nanoseconds / 1e9

        # Convert to compact string
        dist_str = " ".join(f"{d:.3f}" for d in distances)

        self.dist_log_file.write(f"{timestamp:.6f} {dist_str}\n")


        # Skalowanie do 0...255
        dist_norm = (255 * (distances - distances.min()) /
                     (distances.max() - distances.min())).astype(np.uint8)

        # Tworzymy obraz 2D (16 x 1800)
        try:
            image = dist_norm.reshape(self.vertical_layers, self.horizontal_samples)
        except ValueError:
            self.get_logger().error(
                f"Nie mogę uformować obrazu: liczba punktów={len(dist_norm)}, "
                f"oczekiwano {self.vertical_layers * self.horizontal_samples}"
            )
            return

        # Kolorowanie (LUT)
        img_color = cv2.applyColorMap(image, cv2.COLORMAP_JET)

        # Wyświetlanie
        cv2.imshow("LiDAR Depth CV2", cv2.normalize(img_color, None, 0, 255, cv2.NORM_MINMAX))
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = LidarCV2Visualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
