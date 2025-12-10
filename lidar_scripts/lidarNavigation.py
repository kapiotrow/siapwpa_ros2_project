import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

VEL_TOPIC = '/model/samochod/cmd_vel'
LIDAR_TOPIC = '/world/mecanum_drive/model/samochod/link/base_footprint/sensor/velodyne_lidar/scan/points'

class LidarNavigator(Node):
    def __init__(self):
        super().__init__('lidar_navigator')
        self.publisher_ = self.create_publisher(Twist, VEL_TOPIC, 10)
        self.subscription = self.create_subscription(PointCloud2, LIDAR_TOPIC, self.lidar_callback, 10)
        self.timer = self.create_timer(0.1, self.control_callback)
        self.latest_points = None

        # Stałe dla kątów
        self.min_angle = -0.261799  # ~ -15° (w radianach)
        self.max_angle = 0.261799   # ~ +15° (w radianach)

    def lidar_callback(self, msg):
        # Konwersja PointCloud2 do numpy array
        points = np.array([[p[0], p[1], p[2], p[3]] for p in pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)])
        self.latest_points = points

    def control_callback(self):
        if self.latest_points is None:
            return

        # Kąty i rozróżnienie na lewo/prawo/przód
        angles = np.arctan2(self.latest_points[:, 1], self.latest_points[:, 0])  # obliczamy kąty z x i y

        # Warunki dla "przód", "lewo", "prawo"
        front = (angles >= self.min_angle) & (angles <= self.max_angle)  # Kąty od -15° do +15° (przód)
        left = angles < self.min_angle  # Kąty < -15° (lewo)
        right = angles > self.max_angle  # Kąty > +15° (prawo)

        # Filtrujemy punkty w poszczególnych sektorach
        front_points = self.latest_points[front]
        left_points = self.latest_points[left]
        right_points = self.latest_points[right]

        # Minimalne odległości w poszczególnych sektorach
        min_front = np.min(front_points[:, 0]) if len(front_points) > 0 else 10.0
        min_left = np.min(left_points[:, 0]) if len(left_points) > 0 else 10.0
        min_right = np.min(right_points[:, 0]) if len(right_points) > 0 else 10.0

        # Tworzymy wiadomość
        msg = Twist()
        max_speed = 1.0  # Maksymalna prędkość przód/tył
        max_turn = 1.0   # Maksymalna prędkość obrotu

        # Algorytm unikania przeszkód
        if min_front < 1.0:  # Przeszkoda z przodu
            msg.angular.z = 0.0  # Zatrzymanie przód/tył
            if min_left > min_right:
                msg.linear.x = 0.5 * max_turn  # Skręć w lewo
            else:
                msg.linear.x = -0.5 * max_turn  # Skręć w prawo
        else:
            msg.angular.z = max_speed  # Jedź do przodu
            # Skręcanie proporcjonalne do różnicy odległości po bokach
            turn = (min_left - min_right) / max(min_left + min_right, 0.01)
            msg.linear.x = max_turn * np.clip(turn, -1.0, 1.0)

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
