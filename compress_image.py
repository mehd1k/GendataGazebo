#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from image_transport import ImageTransport

class ImageCompressor(Node):

    def __init__(self):
        super().__init__('image_compressor')
        
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw/compressed', 10)
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )
        
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
            _, encoded_image = cv2.imencode('.jpg', cv_image, encode_param)
            compressed_image_msg = self.bridge.cv2_to_compressed_imgmsg(encoded_image)
            self.publisher_.publish(compressed_image_msg)
        except Exception as e:
            self.get_logger().error('Failed to compress image: {}'.format(str(e)))


def main(args=None):
    rclpy.init(args=args)
    image_compressor = ImageCompressor()
    rclpy.spin(image_compressor)
    image_compressor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
