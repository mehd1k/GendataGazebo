#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import json
import numpy as np
import matplotlib.pyplot as plt
import scipy 
import os
from PIL import Image as PILImage
from scipy.signal import find_peaks, argrelextrema
import scipy.ndimage
import cv2



def check_valid_arg(size_mat, arg):
    if arg < 0:
        out = 0
    elif arg >= size_mat:
        out = size_mat - 1
    else:
        out = arg
    return out

def remove_close_elements(arr, threshold):
    result = []
    for elem in arr:
        if all(abs(elem - x) >= threshold for x in result):
            result.append(elem)
    return np.array(result)


class SensorDataSubscriber(Node):

    def __init__(self):
        super().__init__('sensor_data_subscriber')

        # LiDAR Subscription
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            2)
        self.lidar_subscription  # prevent unused variable warning

        # Camera Subscription
        self.camera_subscription = self.create_subscription(
            Image,
            '/my_camera/image_raw',
            self.camera_callback,
            2)
        self.camera_subscription  # prevent unused variable warning

        self.bridge = CvBridge()  # Instance of CvBridge

        ######### Parameters for generating heatmaps
        self.size_img_x = 128
        self.size_img_y = 96
        self.marker_size = 2
        self.camera_fov = 2.0
        self.camera_max_depth = 20

        # Variables to store data
        self.lidar_data = None
        self.camera_data = None

    def lidar_callback(self, msg):
        self.lidar_data = msg
        self.process_data()

    def camera_callback(self, msg):
        self.camera_data = msg
        self.process_data()

    def process_data(self):
        if self.lidar_data is None or self.camera_data is None:
            return  # Ensure we have data from both sensors

        timestamp = self.lidar_data.header.stamp.sec
        nanosec = str(self.lidar_data.header.stamp.nanosec)[0]

        # Process and save LiDAR data
        self.process_lidar_data(timestamp, nanosec)
        
        # Process and save camera data
        self.process_camera_data(timestamp, nanosec)

    def process_lidar_data(self, timestamp, nanosec):
        self.stamp = self.lidar_data.header.stamp.sec
        self.nanosec = str(self.lidar_data.header.stamp.nanosec)[0]
        self.angle_min = self.lidar_data.angle_min
        self.angle_max = self.lidar_data.angle_max
        self.range_min = self.lidar_data.range_min
        self.range_max = self.lidar_data.range_max
        self.ranges = list(self.lidar_data.ranges)
        self.ranges = np.array(self.ranges)
        self.ranges[np.isinf(self.ranges)] = 2 * self.range_max
        self.samples_size = len(self.ranges)
        self.angle_ls = np.linspace(self.angle_min, self.angle_max, self.samples_size)
   
        self.label_img()
        self.plot_corners()
        self.get_logger().info('LiDAR data processed and label saved. Timestamp: ' + str(timestamp))

    def process_camera_data(self, timestamp, nanosec):
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_data, desired_encoding='bgr8')
        filename = '/home/mehdi/data_gazebo/image/img' + str(timestamp) + '.' + nanosec + '.jpg'
        cv2.imwrite(filename, cv_image)
        self.get_logger().info('Camera image saved. Timestamp: ' + str(timestamp))

    def remove_close_elements_dist(self, indices, threshold):
        result = []
        for elem in indices:
            p1 = np.array([self.ranges[elem] * np.cos(self.angle_ls[elem]), self.ranges[elem] * np.sin(self.angle_ls[elem])])
            if all(np.linalg.norm(p1 - np.array([self.ranges[x] * np.cos(self.angle_ls[x]), self.ranges[x] * np.sin(self.angle_ls[x])])) >= threshold for x in result):
                result.append(elem)
        return np.array(result)

    def find_local_extremums(self, signal, threshold=500, window_size=10):
        peaks, _ = find_peaks(signal)
        minima = argrelextrema(signal, np.less)[0]
        extremums = np.hstack((peaks, minima))
        filtered_extremums = [index for index in extremums if signal[index] >= threshold]
        half_window = window_size // 2
        suppressed_peaks = []

        for peak in filtered_extremums:
            left = max(0, peak - half_window)
            right = min(len(signal), peak + half_window + 1)
            if signal[peak] == max(signal[left:right]):
                suppressed_peaks.append(peak)

        fig, ax = plt.subplots()
        ax.plot(self.ranges)
        for arg_peak in suppressed_peaks:
            ax.plot(arg_peak, self.ranges[arg_peak], marker='o', color='red')

        filename = 'dif' + str(self.stamp) + '.png'
        fig.savefig(os.path.join('/home/mehdi/data_gazebo/dif_sig', filename))
        plt.close(fig)
        return np.array(suppressed_peaks)
    


    def plot_corners(self):
       
        # # for local maxima
        # local_maxima_arg = scipy.signal.argrelextrema(self.ranges, np.greater)


        # # for local minima
        # local_minima_arg = scipy.signal.argrelextrema(self.ranges, np.less)
        corner_arg_ls = self.find_corners(self.ranges)

        

        fig, ax = plt.subplots()
        ax.plot(self.angle_ls, self.ranges)
        for arg_peak in corner_arg_ls:
            ax.plot(self.angle_ls[arg_peak],self.ranges[arg_peak],marker = 'o', color = 'red')


        filename='range'+str(self.stamp)+ '.' + self.nanosec +'.png'
        # im1.save(os.path.join('/home/mehdi/data_gazebo/label', filename))
        fig.savefig(os.path.join('/home/mehdi/data_gazebo/range_plots', filename))
        plt.close(fig) 
        # fig.savefig('rangeplt.png')


    def find_corners(self, signal, sigma=1, threshold=450):
        smooth_signal = signal
        first_diff = np.abs(np.gradient(smooth_signal, self.angle_ls))
        second_diff = np.abs(np.gradient(np.gradient(smooth_signal, self.angle_ls), self.angle_ls))
        discontinuities_second = self.find_local_extremums(second_diff)
        discontinuities_second = self.remove_close_elements_dist(discontinuities_second, 0.5)
        return discontinuities_second

    def label_img(self):
        corner_arg_ls = self.find_corners(self.ranges)
        self.corners_ls = []
        for arg in corner_arg_ls:
            arg_prior = max(0, arg - 1)
            arg_next = min(arg + 1, self.samples_size)
            rng = min(self.ranges[arg], self.ranges[arg_prior], self.ranges[arg_next])
            self.corners_ls.append(np.array([self.angle_ls[arg], rng]))
        img_mat = np.zeros((self.size_img_y, self.size_img_x))
        img_mat.astype(int)

        for corner in self.corners_ls:
            p = self.pos_in_image(corner)
            l1, l2 = check_valid_arg(self.size_img_x, int(p[0] - self.marker_size / 2)), check_valid_arg(self.size_img_x, int(p[0] + self.marker_size / 2))
            v1, v2 = check_valid_arg(self.size_img_y, int(p[1] - self.marker_size / 2)), check_valid_arg(self.size_img_y, int(p[1] + self.marker_size / 2))
            temp = img_mat[v1:v2, l1:l2]
            img_mat[v1:v2, l1:l2] = np.ones(temp.shape)

        fig, ax = plt.subplots()
        ax = plt.imshow(img_mat)
        filename = 'label' + str(self.stamp) + '.' + self.nanosec + '.png'
        fig.savefig(os.path.join('/home/mehdi/data_gazebo/label', filename))
        plt.close(fig)
        np.save(os.path.join('/home/mehdi/data_gazebo/label_npy', filename), img_mat)
        return img_mat

    def pos_in_image(self, point):
        xrange_min = -self.camera_fov / 2
        xrange_max = self.camera_fov / 2
        resolution_x = (xrange_max - xrange_min) / self.size_img_x

        yrange_max = self.camera_max_depth
        yrange_min = 0
        resolution_y = (yrange_max - yrange_min) / self.size_img_y

        out_x = np.floor((point[0] - xrange_min) / resolution_x)
        out_y = np.floor((point[1] - yrange_min) / resolution_y)

        return [out_x, out_y]


def main(args=None):
    rclpy.init(args=args)
    sensor_data_subscriber = SensorDataSubscriber()
    rclpy.spin(sensor_data_subscriber)
    sensor_data_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
