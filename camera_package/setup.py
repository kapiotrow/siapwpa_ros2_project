from setuptools import find_packages, setup

package_name = 'camera_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
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
            'talker = camera_package.publisher_member_function:main',
            'listener = camera_package.subscriber_member_function:main'
        ],
    },
)
