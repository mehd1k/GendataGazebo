#!/usr/bin/env python3 
import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan
import json
import numpy as np
import matplotlib.pyplot as plt
import scipy 
import os
from PIL import Image
from scipy.signal import find_peaks, argrelextrema
import scipy.ndimage
import glob

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



class LidarProcess():

    def __init__(self, liadr_data_dir):

        #########Parameters for generating heatmaps
        self.counter = 0
        self.size_img_x = 128
        self.size_img_y =96
        self.marker_size = 2
        self.camera_fov = 2.0
        self.camera_max_depth = 20
        self.angle_min = -0.93
        self.angle_max =  0.93
        self.liadr_data_dir = liadr_data_dir
    

    def get_range_check(self, id, range_len = 1):
        if id -range_len >= 0 and id+(range_len+1)<len(self.ranges):
            out = range(id-range_len, id+(range_len+1))
            return out
        elif id -range_len< 0:
            out = range(0, id+(range_len+1))
            return out
        elif id+(range_len+1)>=len(self.ranges):
            out = range(id-range_len, len(self.ranges))
            return out


    def remove_close_elements_dist(self,indices, threshold):
        """
        Remove elements from the array that are within the threshold distance
        from any other element.

        :param arr: Input array of integers.
        :param threshold: Threshold distance.
        :return: Array with close elements removed.
        """
        groups = []
        if len(indices)== 0:
            min_discontinuities = []
        else:
            current_group = [indices[0]]

            for i in range(1, len(indices)):
                p1 = np.array([self.ranges[indices[i]]*np.cos(self.angle_ls[indices[i]]), self.ranges[indices[i]]*np.sin(self.angle_ls[indices[i]])])
                p2 = np.array([self.ranges[indices[i-1]]*np.cos(self.angle_ls[indices[i-1]]), self.ranges[indices[i-1]]*np.sin(self.angle_ls[indices[i-1]])])
                if np.linalg.norm(p1-p2)<= threshold and np.abs(indices[i]-indices[i-1])<=50:
                    current_group.append(indices[i])
                else:
                    groups.append(current_group)
                    current_group = [indices[i]]
            groups.append(current_group)

            # Select the point with the minimum signal value in each group
            min_discontinuities = []

            for group in groups:
                min_point = min(group, key=lambda x: self.ranges[x])
                min_discontinuities.append(min_point)















        # result = []
        # for elem in indices:
        #     flag = True
        #     range_check2 = self.get_range_check(elem)
        #     p1 = np.array([self.ranges[elem]*np.cos(self.angle_ls[elem]), self.ranges[elem]*np.sin(self.angle_ls[elem])])
        #     if any(np.linalg.norm(p1 - np.array([self.ranges[x]*np.cos(self.angle_ls[x]), self.ranges[x]*np.sin(self.angle_ls[x])])) <= threshold for x in result):
        #         flag = False
        #     # else:
        #     #     for x in result:
        #     #         range_check = self.get_range_check(x)

        #     #         for ri in range_check:
        #     #             for ri2 in range_check2: 
        #     #                 p2 = np.array([self.ranges[ri2]*np.cos(self.angle_ls[ri2]), self.ranges[ri2]*np.sin(self.angle_ls[ri2])])
        #     #                 if np.linalg.norm(p2 - np.array([self.ranges[ri]*np.cos(self.angle_ls[ri]), self.ranges[ri]*np.sin(self.angle_ls[ri])])) <= threshold:
        #     #                     flag = False
        #     #                     break
                    


        #     if flag:                
        #         result.append(elem)
    
        return np.array(min_discontinuities)
    def save_corners(self, id,file_name):
        self.id = id
        self.ranges = np.load(os.path.join(self.liadr_data_dir,file_name))[::-1]
        self.samples_size = len(self.ranges) 
        #### angle_samples
        self.angle_ls =  np.linspace(self.angle_min,self.angle_max,self.samples_size)
        self.corner_arg_ls = self.find_corners(self.ranges)

    def find_local_extremums(self, signal, threshold = 150,window_size = 10):


       

        # Find peaks
        peaks, _ = find_peaks(signal)

        # Find local minima
        minima = argrelextrema(signal, np.less)[0]

        extremums = np.hstack((peaks,minima))
        
        extremums = np.sort(extremums)
        filtered_extremums = [index for index in extremums if np.abs(signal[index]) >= threshold]
        # filtered_extremums = extremums


# Assuming `signal` is your input signal array and `derivative` is its derivative
# Compute the derivative if not already available



        distance_threshold = 10
        # Group close points
        groups = []
        if len(filtered_extremums)== 0:
            suppressed_peaks = []
        else:
            current_group = [filtered_extremums[0]]

            for i in range(1, len(filtered_extremums)):
                if filtered_extremums[i] - filtered_extremums[i-1] <= distance_threshold:
                    current_group.append(filtered_extremums[i])
                else:
                    groups.append(current_group)
                    current_group = [filtered_extremums[i]]
            groups.append(current_group)

            # Select the point with the minimum signal value in each group
            min_discontinuities = []

            for group in groups:
                min_point = min(group, key=lambda x: self.ranges[x])
                min_discontinuities.append(min_point)

            # `min_discontinuities` now contains the indices of the discontinuous points
            suppressed_peaks = min_discontinuities

        
        
        
        
        
        
       
       
       
       
       
       
       
       
       
       
       
       
       
       
       
       
       
        # half_window = window_size // 2
        # suppressed_peaks = []

        # for peak in filtered_extremums:
        #     left = max(0, peak - half_window)
        #     right = min(len(signal), peak + half_window + 1)
        #     if self.ranges[peak] == min(self.ranges[left:right]):
        #         suppressed_peaks.append(peak)

        fig, ax = plt.subplots(2)
        ax[0].plot(signal)
        ax[1].plot(self.ranges)
        for arg_peak in suppressed_peaks:
            ax[0].plot(arg_peak,self.ranges[arg_peak],marker = 'o', color = 'red')
            ax[1].plot(arg_peak,self.ranges[arg_peak],marker = 'o', color = 'red')


        filename='dif'+str(self.id)+'.png'
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
        

        

        fig, ax = plt.subplots()
        ax.plot(self.angle_ls, self.ranges)
        for arg_peak in self.corner_arg_ls:
            ax.plot(self.angle_ls[arg_peak],self.ranges[arg_peak],marker = 'o', color = 'red')


        filename='range'+id+'.png'
        # im1.save(os.path.join('/home/mehdi/data_gazebo/label', filename))
        fig.savefig(os.path.join('/home/mehdi/data_gazebo/range_plots', filename))
        plt.close(fig) 
        # fig.savefig('rangeplt.png')


        
        
    def label_img(self):
        ########## genrating heatmap images
      
        self.corners_ls=[]
        for arg in self.corner_arg_ls:
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
        filename='label'+id+'.png'
        # im1.save(os.path.join('/home/mehdi/data_gazebo/label', filename))
        fig.savefig(os.path.join('/home/mehdi/data_gazebo/label', filename))
        plt.close(fig) 
        filename_npy='label'+id+'.npy'
        np.save( os.path.join('/home/mehdi/data_gazebo/label_npy', filename_npy),img_mat)
        
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


# save_label_npy_dir = '/home/mehdi/data_gazebo/label_npy'
# save_label_img_dir =  '/home/mehdi/data_gazebo/label'
# save_plt_signal_dir = '/home/mehdi/data_gazebo/range_plots'
liadr_data_dir = '/home/mehdi/data_gazebo/scan_data'

pc = LidarProcess(liadr_data_dir)

###### Get a list of all files in the directory
files = [f for f in os.listdir(liadr_data_dir) if os.path.isfile(os.path.join(liadr_data_dir, f))]
counter = 1
files.sort()
for f in files: 
    print('file ', str(counter), 'from', str(len(files)))
    id = f.split('.npy')[0].split('range')[1]
    if id =='2114.4':
        print(id)
    pc.save_corners(id,f)
    pc.plot_corners()
    pc.label_img()
    counter += 1

# id = '9737.6'
# f = 'range'+id+'.npy'
# pc.save_corners(id,f)
# pc.plot_corners()
# pc.label_img()
