from setuptools import find_packages, setup

package_name = 'robot_agent'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yourusername',
    maintainer_email='your.email@example.com',
    description='ROS 2 Robot Agent for Robot Management System',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mqtt_ros_bridge = robot_agent.mqtt_ros_bridge:main',
        ],
    },
)
