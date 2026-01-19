import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from .line_follower import LineFollower
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from pynput import keyboard as kb
from sensor_msgs.msg import LaserScan
from . obstacle_avoidance import ObstacleAvoidance
import mecanum_pb2
import serial
import time

VEL_TOPIC = '/model/samochod/cmd_vel'
TOPIC_RGB = '/world/mecanum_drive/model/samochod/link/base_footprint/sensor/realsense_rgbd/image'
TOPIC_DEPTH = '/world/mecanum_drive/model/samochod/link/base_footprint/sensor/realsense_depth/depth_image'
TOPIC_LIDAR = '/world/mecanum_drive/model/samochod/link/base_footprint/sensor/velodyne_lidar/scan'
bridge = CvBridge()


class VelocityPublisher(Node):

    def __init__(self):
        super().__init__('velocity_publisher')

        try:
            self.ser = serial.Serial('/dev/ttyUSB1', 19200, timeout=1)  # Dostosuj port według potrzeb
            time.sleep(2)
            print("Połączono z robotem przez port /dev/ttyUSB1")
            self.robot_connected = True
        except serial.SerialException:
            print("Nie można połączyć z robotem - sprawdź połączenie USB")
            self.robot_connected = False
            self.ser = None

        self.publisher_ = self.create_publisher(Twist, VEL_TOPIC, 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.camera_subscriber = self.create_subscription(
            Image,
            TOPIC_RGB,
            self.camera_callback,
            10
        )

        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            TOPIC_LIDAR,
            self.lidar_callback,
            10
        )

        self.max_speed_forward = 1.0  # m/s
        self.max_speed_sideways = 4.0 # m/s

        self.obstacle_avoider = ObstacleAvoidance()
        self.obstacle_directions_vector = [0.0, 0.0, 0.0]
        self.isAvoiding = self.obstacle_directions_vector[2]

        self.line_follower = LineFollower()
        self.directions_vector = [0.0, 0.0, 0.0]
        self.manual = False
        self.keys = set()

        self.listener = kb.Listener(
            on_press=self.on_press,
            on_release=self.on_release
            )
        self.listener.start()

    def on_press(self, key):
        try:
            if key.char == 'm':    # toggle manual mode
                self.manual = not self.manual
                mode = "MANUAL" if self.manual else "AUTOMATIC"
                self.get_logger().info(f"Switched to {mode}")
            else:
                self.keys.add(key.char)
        except AttributeError:
            # arrow keys & specials
            self.keys.add(key)

    def on_release(self, key):
        try:
            self.keys.discard(key.char)
        except AttributeError:
            self.keys.discard(key)


    def camera_callback(self, msg):
        cv_image = bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.directions_vector = self.line_follower.update(cv_image)


    def lidar_callback(self, msg):
        self.obstacle_directions_vector = self.obstacle_avoider.update(msg)


    def timer_callback(self):
        self.isAvoiding = False

        if self.manual:
            msg = Twist()

            # forward/back
            if 'w' in self.keys:
                msg.angular.z  = -self.max_speed_forward//2
            elif 's' in self.keys:
                msg.angular.z  = self.max_speed_forward//2

            # left/right
            if 'a' in self.keys:
                msg.linear.x = -self.max_speed_sideways
            elif 'd' in self.keys:
                msg.linear.x = self.max_speed_sideways

            self.publisher_.publish(msg)

            request = mecanum_pb2.ControlRequest()
            request.speed_mmps = int(msg.angular.z)
            request.rad = float(msg.linear.z)
            request.omega = float(0)
        
            # Serializacja wiadomości
            serialized_data = request.SerializeToString()
            
            # Wysłanie do Arduino
            self.ser.write(serialized_data)

        elif self.isAvoiding:
            self.get_logger().info(f"AVOIDING")
            visibility = self.obstacle_directions_vector[0]
            offset = self.obstacle_directions_vector[1]

            msg = Twist()
            msg.linear.x = self.max_speed_sideways * offset
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = -self.max_speed_forward * visibility

            self.publisher_.publish(msg)

        else:
            visibility = self.directions_vector[0]
            offset = self.directions_vector[1]

            msg = Twist()
            msg.linear.x = self.max_speed_sideways * offset
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = -self.max_speed_forward * visibility

            self.publisher_.publish(msg)

            request = mecanum_pb2.ControlRequest()
            request.speed_mmps = int(msg.angular.z)
            request.rad = float(msg.linear.z)
            request.omega = float(0)
        
            # Serializacja wiadomości
            serialized_data = request.SerializeToString()
            
            # Wysłanie do Arduino
            self.ser.write(serialized_data)



def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

  
if __name__ == '__main__':
    main()