from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Bridge dla RGB
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='rgb_camera_bridge',
            output='screen',
            arguments=[
                '/world/mecanum_drive/model/samochod/link/base_footprint/sensor/realsense_rgbd/image'
                '@sensor_msgs/msg/Image@gz.msgs.Image'
            ]
        ),

        # Bridge dla Depth
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='depth_camera_bridge',
            output='screen',
            arguments=[
                '/world/mecanum_drive/model/samochod/link/base_footprint/sensor/realsense_depth/depth_image'
                '@sensor_msgs/msg/Image@gz.msgs.Image'
            ]
        ),

        # Bridge dla cmd_vel
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='cmd_vel_bridge',
            output='screen',
            arguments=[
                '/model/samochod/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
            ]
        ),

        # Bridge dla Velodyne LiDAR LaserScan
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='velodyne_lidar_scan_bridge',
            output='screen',
            arguments=[
                '/world/mecanum_drive/model/samochod/link/base_footprint/sensor/velodyne_lidar/scan'
                '@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'
            ]
        ),

        # Bridge dla Velodyne LiDAR PointCloud2 
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='velodyne_lidar_points_bridge',
            output='screen',
            arguments=[
                '/world/mecanum_drive/model/samochod/link/base_footprint/sensor/velodyne_lidar/scan/points'
                '@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'
            ]
        )
    ])
