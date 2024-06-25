#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import os



class LaserScanSubscriber(Node):

    def __init__(self):
        super().__init__('laser_scan_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warni
        timer_period = 0.3  # seconds (10 Hz)
        self.timer = self.create_timer(timer_period, self.process_callback)
        self.counter = 0
       

    def listener_callback(self, msg):
        self.stamp= msg.header.stamp.sec
        self.nanosec = str(msg.header.stamp.nanosec)[0]
        self.range_max = msg.range_max
        self.ranges = list(msg.ranges)
        

    def process_callback(self):
        self.ranges = np.array(self.ranges)
        self.ranges[ np.isinf(self.ranges)] = 2*self.range_max
        filename='range'+str(self.stamp)+'.'+self.nanosec
        np.save(os.path.join('/home/mehdi/data_gazebo/scan_data', filename), self.ranges)
        self.get_logger().info('Label saved stamp = '+str(self.stamp)+'  counter = '+str(self.counter))
        self.counter +=1

        

    # def listener_callback(self, msg):
      
    #     self.stamp= msg.header.stamp.sec
    #     self.nanosec = str(msg.header.stamp.nanosec)[0]
        
    #     self.range_max = msg.range_max
    #     self.ranges = list(msg.ranges)
    #     self.ranges = np.array(self.ranges)
    #     self.ranges[ np.isinf(self.ranges)] = 2*self.range_max
        


    #     self.save_data()
    #     self.counter +=1
      
    #     self.get_logger().info('Label saved stamp = '+str(self.stamp)+'  counter = '+str(self.counter))
    

    # def save_data(self):

    #     filename='range'+str(self.stamp)+'.'+self.nanosec
    #     np.save(os.path.join('/home/mehdi/data_gazebo/scan_data', filename), self.ranges)



def main(args=None):
    rclpy.init(args=args)
    laser_scan_subscriber = LaserScanSubscriber()
    rclpy.spin(laser_scan_subscriber)
    laser_scan_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
