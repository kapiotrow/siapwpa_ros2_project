from setuptools import find_packages, setup
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from glob import glob


package_name = 'camera_package'

def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            file_path = os.path.join(path, filename)
            install_path = os.path.join(
                'share',
                package_name,
                path
            )
            paths.append((install_path, [file_path]))
    return paths


data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name + '/launch', ['launch/bridge.launch.py']),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'worlds'),
        glob('worlds/*.sdf')),
]

data_files += package_files('models')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # data_files=[
    #     ('share/ament_index/resource_index/packages',
    #         ['resource/' + package_name]),
    #     ('share/' + package_name, ['package.xml']),
    #     ('share/' + package_name + '/launch', ['launch/bridge.launch.py']),
    #     # Worlds
    #     (os.path.join('share', package_name, 'worlds'),
    #         glob('worlds/*.sdf')),

    #     # Models (recursive)
    #     (os.path.join('share', package_name, 'models'),
    #         glob('models/**/*', recursive=True)),
    # ],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='kapiotrow@student.agh.edu.pl',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'my_node = camera_package.my_node:main',
            # 'camera_subscriber = camera_package.camera_subscriber:main'
            # 'talker = camera_package.publisher_member_function:main',
            # 'listener = camera_package.subscriber_member_function:main'
            'camera_pubsub = camera_package.camera_pubsub:main',
            'pose_info_subscriber = camera_package.position_subscriber:main'
        ],
    },
)
