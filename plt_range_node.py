#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import json
import numpy as np
import matplotlib.pyplot as plt
import scipy 
import os
from PIL import Image
from scipy.signal import find_peaks, argrelextrema
import scipy.ndimage


def check_valid_arg(size_mat, arg):
    if arg<0:
        out=0
    elif arg>=size_mat:
        out=size_mat-1
    else:
        out=arg
    return out

def remove_close_elements(arr, threshold):
    """
    Remove elements from the array that are within the threshold distance
    from any other element.

    :param arr: Input array of integers.
    :param threshold: Threshold distance.
    :return: Array with close elements removed.
    """
    result = []
    for elem in arr:
        if all(abs(elem - x) >= threshold for x in result):
            result.append(elem)
    return np.array(result)



class LaserScanSubscriber(Node):

    def __init__(self):
        super().__init__('laser_scan_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            2)
        self.subscription  # prevent unused variable warning


        #########Parameters for generating heatmaps
        
        self.size_img_x = 128
        self.size_img_y =96
        self.marker_size = 2
        self.camera_fov = 2.0
        self.camera_max_depth = 20
        

    def listener_callback(self, msg):
        # laser_scan_data = {
        #     "header": {
        #         "stamp": msg.header.stamp.sec,  # or .nanosec for more precision
        #         "frame_id": msg.header.frame_id
        #     },
        #     "angle_min": msg.angle_min,
        #     "angle_max": msg.angle_max,
        #     "angle_increment": msg.angle_increment,
        #     "time_increment": msg.time_increment,
        #     "scan_time": msg.scan_time,
        #     "range_min": msg.range_min,
        #     "range_max": msg.range_max,
        #     "ranges": list(msg.ranges),
        #     "intensities": list(msg.intensities)
        # }
       
        self.stamp= msg.header.stamp.sec
        self.nanosec = str(msg.header.stamp.nanosec)[0]
        self.angle_min = msg.angle_min
        self.angle_max =  msg.angle_max
        self.range_min = msg.range_min
        self.range_max = msg.range_max
        self.ranges = list(msg.ranges)
        self.ranges = np.array(self.ranges)
        self.ranges[ np.isinf(self.ranges)] = 2*self.range_max
        self.samples_size = len(self.ranges) 
        #### angle_samples
        self.angle_ls =  np.linspace(self.angle_min,self.angle_max,self.samples_size)

        self.plot_corners()
        # self.label_img()
        # filename = 'laser_scan_data'+str(self.stamp)+'.json'
        # Save data to a file
        # save_dir = os.path.join('/home/mehdi/data_gazebo/scan_data', filename)
        # with open(save_dir, 'w') as outfile:
        #     json.dump(laser_scan_data, outfile)

        self.get_logger().info('Label saved stamp = '+str(self.stamp))
    
    
    def remove_close_elements_dist(self,indices, threshold):
        """
        Remove elements from the array that are within the threshold distance
        from any other element.

        :param arr: Input array of integers.
        :param threshold: Threshold distance.
        :return: Array with close elements removed.
        """
        result = []
        for elem in indices:
            p1 = np.array([self.ranges[elem]*np.cos(self.angle_ls[elem]), self.ranges[elem]*np.sin(self.angle_ls[elem])])
            if all(np.linalg.norm(p1 - np.array([self.ranges[x]*np.cos(self.angle_ls[x]), self.ranges[x]*np.sin(self.angle_ls[x])])) >= threshold for x in result):
                result.append(elem)
        return np.array(result)
    def find_local_extremums(self, signal, threshold = 500,window_size = 10):


       

        # Find peaks
        peaks, _ = find_peaks(signal)

        # Find local minima
        minima = argrelextrema(signal, np.less)[0]

        extremums = np.hstack((peaks,minima))
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
            ax.plot(arg_peak,self.ranges[arg_peak],marker = 'o', color = 'red')


        filename='dif'+str(self.stamp)+'.png'
        # im1.save(os.path.join('/home/mehdi/data_gazebo/label', filename))
        fig.savefig(os.path.join('/home/mehdi/data_gazebo/dif_sig', filename))
        plt.close(fig) 
        return np.array(suppressed_peaks)

    def find_corners(self, signal, sigma=1, threshold=450):
        """
        Find first oreder and second order discontinuities in a signal.

        :param signal: Input signal as a NumPy array.
        :param sigma: Standard deviation for Gaussian kernel used in smoothing.
        :param threshold: Threshold for detecting discontinuities in the gradient.
        :return: Indices of discontinuities.
        """


        # Calculate the gradient of the smoothed signal
        smooth_signal = signal
        first_diff = np.abs(np.gradient(smooth_signal, self.angle_ls))
        second_diff = np.abs(np.gradient(np.gradient(smooth_signal, self.angle_ls), self.angle_ls))

        # Identify points where the gradient is above a threshold
        # discontinuities_first = np.where(first_diff > threshold1)[0] + 1  # +1 for the offset caused by np.diff
        discontinuities_second =  self.find_local_extremums(second_diff)
        ###removing repated args or very close args
        # discontinuities_second = remove_close_elements(discontinuities_second, 10)
        discontinuities_second = self.remove_close_elements_dist(discontinuities_second, 0.5)


        return discontinuities_second
        # return np.hstack((discontinuities_first, discontinuities_second))
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


        filename='range'+str(self.stamp)+'.'+self.nanosec+'.png'
        # im1.save(os.path.join('/home/mehdi/data_gazebo/label', filename))
        fig.savefig(os.path.join('/home/mehdi/data_gazebo/range_plots', filename))
        plt.close(fig) 
        # fig.savefig('rangeplt.png')


        
        
    def label_img(self):
        ########## genrating heatmap images
        corner_arg_ls = self.find_corners(self.ranges)
        self.corners_ls=[]
        for arg in corner_arg_ls:
            arg_prior = max(0, arg-1)
            arg_next = min(arg+1, self.samples_size)
            rng = min(self.ranges[arg],self.ranges[arg_prior],self.ranges[arg_next])
            self.corners_ls.append(np.array([self.angle_ls[arg], rng]))
        # first f.corners_ls.append(np.array([rng, self.ranges[arg]]))
        # first channel for corners
        img_mat=np.zeros((self.size_img_y,self.size_img_x))
        img_mat.astype(int)
         
        
        for corner in self.corners_ls:
            p=self.pos_in_image(corner)
            l1, l2=check_valid_arg(self.size_img_x, int( p[0]-self.marker_size/2)), check_valid_arg(self.size_img_x,int(p[0]+self.marker_size/2))
            v1, v2=check_valid_arg(self.size_img_y,int(p[1]-self.marker_size/2)), check_valid_arg(self.size_img_y,int(p[1]+self.marker_size/2))
            temp= img_mat[v1:v2, l1:l2]                        
            img_mat[v1:v2, l1:l2] = np.ones(temp.shape)

        
        fig, ax = plt.subplots()
        # im1.show()
        ax = plt.imshow(img_mat)
        filename='label'+str(self.stamp)+'.png'
        # im1.save(os.path.join('/home/mehdi/data_gazebo/label', filename))
        fig.savefig(os.path.join('/home/mehdi/data_gazebo/label', filename))
        plt.close(fig) 
        np.save( os.path.join('/home/mehdi/data_gazebo/label_npy', filename),img_mat)
        
        # indx_pic=indx_pic+1

        return img_mat
    def pos_in_image(self,point):
        xrange_min = -self.camera_fov/2
        xrange_max = self.camera_fov/2
        resolution_x = (xrange_max-xrange_min)/self.size_img_x
        
        yrange_max = self.camera_max_depth
        yrange_min = 0 
        resolution_y = (yrange_max-yrange_min)/self.size_img_y
     
        out_x=np.floor((point[0]-xrange_min)/resolution_x)
        out_y=np.floor((point[1]-yrange_min)/resolution_y) 
     
        return [out_x, out_y]


def main(args=None):
    rclpy.init(args=args)
    laser_scan_subscriber = LaserScanSubscriber()
    rclpy.spin(laser_scan_subscriber)
    laser_scan_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
