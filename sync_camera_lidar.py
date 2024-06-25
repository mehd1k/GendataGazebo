#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from message_filters import Subscriber, ApproximateTimeSynchronizer

class SynchronizerNode(Node):

    def __init__(self):
        super().__init__('synchronizer_node')

        self.image_sub = Subscriber(self, Image, '/my_camera/image_raw')
        self.lidar_sub = Subscriber(self, LaserScan, '/scan')

        self.ts = ApproximateTimeSynchronizer([self.image_sub, self.lidar_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.sync_callback)

        self.bridge = CvBridge()
        self.image_data = None
        self.lidar_data = None

        # Set the desired frequency (in Hz)
        # frequency = 3  # Change this to your desired frequency
        timer_period = 0.3
        self.timer = self.create_timer(timer_period, self.process_callback)

    def sync_callback(self, image_msg, lidar_msg):
        self.image_data = image_msg
        self.lidar_data = lidar_msg

    def process_callback(self):
        if self.image_data is not None and self.lidar_data is not None:
            sec = self.image_data.header.stamp.sec
            nanosec = self.image_data.header.stamp.nanosec

            # Save image
            cv_image = self.bridge.imgmsg_to_cv2(self.image_data, desired_encoding='passthrough')
            image_filename = f'/home/mehdi/data_gazebo/image/img{sec}.{str(nanosec)[0]}.jpg'
            cv2.imwrite(image_filename, cv_image)
            self.get_logger().info(f'Image saved: {image_filename}')

            # Save lidar data
            lidar_filename = f'/home/mehdi/data_gazebo/scan_data/range{sec}.{str(self.lidar_data.header.stamp.nanosec)[0]}.npy'
            ranges = np.array(self.lidar_data.ranges)
            ranges[np.isinf(ranges)] = 2 * self.lidar_data.range_max
            np.save(lidar_filename, ranges)
            self.get_logger().info(f'Lidar data saved: {lidar_filename}')

            # Reset data after processing
            self.image_data = None
            self.lidar_data = None

def main(args=None):
    rclpy.init(args=args)
    synchronizer_node = SynchronizerNode()
    rclpy.spin(synchronizer_node)
    synchronizer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
