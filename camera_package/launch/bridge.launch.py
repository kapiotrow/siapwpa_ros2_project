import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '/home/developer/ros2_ws/src/install/camera_package/share/camera_package/worlds/racetrack.sdf'}.items(),
    )


    bridges = [
        # RGB camera bridge
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

        # Depth camera bridge
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

        # cmd_vel bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='cmd_vel_bridge',
            output='screen',
            arguments=[
                '/model/samochod/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
            ]
        ),

        # LiDAR LaserScan
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

        # LiDAR PointCloud2
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='velodyne_lidar_points_bridge',
            output='screen',
            arguments=[
                '/world/mecanum_drive/model/samochod/link/base_footprint/sensor/velodyne_lidar/scan/points'
                '@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'
            ]
        ),

        # Pose info
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='pose_info_bridge',
            output='screen',
            arguments=[
                '/world/mecanum_drive/pose/info'
                '@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V'
            ]
        ),
    ]



    camera_pubsub_node = Node(
        package='camera_package',
        executable='camera_pubsub',
        name='camera_pubsub',
        output='screen'
    )

    position_subscriber_node = Node(
            package='camera_package',
            executable='pose_info_subscriber',
            name='pose_info_subscriber',
            output='screen'
    )

    lidar_subscriber_node = Node(
            package='camera_package',
            executable='lidar_subscriber',
            name='lidar_subscriber',
            output='screen'
    )

    return LaunchDescription([

        # Start Gazebo immediately
        gz_sim,

        # Start bridges after Gazebo (1 second delay)
        TimerAction(
            period=1.0,
            actions=bridges
        ),

        # Start pub/sub after everything is ready
        TimerAction(
            period=3.0,
            actions=[camera_pubsub_node]
        ),

        TimerAction(
            period=0.5,
            actions=[position_subscriber_node]
        ),

        TimerAction(
            period=0.5,
            actions=[lidar_subscriber_node]
        )
    ])