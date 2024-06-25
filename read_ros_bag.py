import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import subprocess

def read_rosbag(bag_path):
    rclpy.init()

    node = rclpy.create_node('rosbag_reader')

    # Create subscribers to process the messages
    node.create_subscription(LaserScan, '/scan', lambda msg: process_message('/scan', msg), 10)
    node.create_subscription(Image, '/my_camera/image_raw', lambda msg: process_message('/my_camera/image_raw', msg), 10)

    # Use subprocess to play the rosbag
    subprocess.run(['ros2', 'bag', 'play', bag_path])

    rclpy.spin(node)
    rclpy.shutdown()

def process_message(topic, msg):
    if topic == '/scan':
        process_laser_scan(msg)
    elif topic == '/my_camera/image_raw':
        process_image(msg)

def process_laser_scan(msg):
    print(f'LaserScan message at {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')
    # Add your processing code here

def process_image(msg):
    print(f'Image message at {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')
    # Add your processing code here

if __name__ == '__main__':
    bag_path = '/path/to/your/rosbag'
    read_rosbag(bag_path)
