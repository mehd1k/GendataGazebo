#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class camera_node(Node):
    def __init__(self):
        super().__init__("camera_node")
        self.subscription = self.create_subscription(Image, '/my_camera/image_raw', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        timer_period = 0.3  # seconds (10 Hz)
        self.timer = self.create_timer(timer_period, self.process_callback)
        self.image_data = None

    def listener_callback(self, msg):
        self.image_data = msg
        self.sec = msg.header.stamp.sec
        self.nansec = msg.header.stamp.nanosec

    def process_callback(self):
        if self.image_data is not None:
            # Process the image data here
            cv_image = self.bridge.imgmsg_to_cv2(self.image_data, desired_encoding='passthrough')
            cv2.imwrite('/home/mehdi/data_gazebo/image/img'+str(self.sec)+'.'+str(self.nansec)[0]+'.jpg', cv_image)
            self.get_logger().info('Image saved'+str(self.sec)+'.'+str(self.nansec)[0])

            # Perform any processing on the cv_image

def main(args=None):
    ###intializing ROS2 communication
    rclpy.init(args=args)
    camera_subscriber = camera_node()
    rclpy.spin(camera_subscriber)
    camera_subscriber.destroy_node()
    rclpy.shutdown()




    rclpy.shutdown()


if __name__ == "__main__":
    main()