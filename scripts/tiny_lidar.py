#!/usr/bin/python
#tinylidar.py
#For the hokuyo urg tiny lidar
#Mark Kocherovsky, Joseph Schulte, May 2022

import rospy
import numpy as np
from math import pi
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from dynamic_reconfigure.server import Server
from actor_lidar.cfg import TinyLidarConfig

class TinyLidar():
	def __init__(self):
		self.li_in = rospy.Subscriber("/tiny_scan", LaserScan, self.laser_callback) #sub to laser scan topic
		self.scan_out = rospy.Publisher('/vehicle/tiny_lidar_smoothed', LaserScan, queue_size = 10) #publish smoothed scan
		self.point_out = rospy.Publisher('/vehicle/closest_tiny_lidar/', Float64, queue_size = 1) #publish to desired topic
		self.fov = 30*(pi/180)
		self.decay = 0.9
		self.thresh = 0.5
		self.window = 1
		self.window_thresh = 0.1
		self.curScan = None
		self.maxrange = 10
		#prepare message
		self.closest_point = -1.0
		self.rate = rospy.Rate(20)
		
		srv = Server(TinyLidarConfig, self.dyncb)
  
		while not rospy.is_shutdown():
			msg = Float64() #initialize and publish
			msg = self.closest_point
			self.point_out.publish(msg)
			self.rate.sleep()
		return
		
	def dyncb(self, config, level):
		self.decay = config.decay
		self.thresh = config.threshold
		self.window = config.window
		self.window_thresh = config.window_thresh
		return config
		
	def laser_callback(self, data):
		range_list = np.array(data.ranges)
		range_list[np.isnan(range_list)] = self.maxrange
		range_list[np.isinf(range_list)] = self.maxrange
		range_list[range_list < 0.1] = self.maxrange
		self.curScan = range_list.copy()
		#if self.curScan is None:
			#self.curScan = range_list
		#else:
			#self.curScan = (self.curScan + self.decay*range_list)/(1 + self.decay)
			#range_temp = range_list
			#range_temp[np.isnan(range_temp)] = self.curScan[np.isnan(range_temp)]
			#range_temp[np.isinf(range_temp)] = range_list[np.isinf(range_temp)]
			#dif = np.abs(self.curScan - range_temp)
			#range_temp[dif > self.thresh] = self.maxrange
			#range_temp = (range_temp + self.decay*self.curScan)/(1 + self.decay)
			#for i in range(range_temp.shape[0]):
			#	i0 = max(0, i - self.window)
			#	i1 = min(range_temp.shape[0] - 1, i + self.window)
			#	if np.any(np.abs(range_temp[i0:i1] - range_temp[i]) > self.window_thresh):
			#		range_temp[i] = np.PINF
			#self.curScan = range_temp.copy()
		
		#range_list = self.curScan.copy()
		i0 = round(range_list.shape[0]/2 - self.fov/(2*data.angle_increment))
		i1 = round(range_list.shape[0]/2 + self.fov/(2*data.angle_increment))
		ic = round(range_list.shape[0]/2) # center angle
		range_list = range_list[i0:i1] #Constrain FOV 
		range_list = range_list[~np.isnan(range_list)] #removes nans
		range_list = range_list[range_list > 0.1] # Cull points that are too close
		data_list = np.array(range_list) #convert data to np array for convenience
		self.closest_point = np.min(data_list) if data_list.shape[0] > 0 else np.PINF #get min point
		#self.closest_point = self.closest_point if self.closest_point >= 0.1 else np.PINF #constrain to 1 cm - Mark
		
		# republish
		tmp = data
		tmp.ranges = self.curScan
		self.scan_out.publish(tmp)
		
		return
		
if __name__ == '__main__':
    
    # Initialize the node and name it.
    rospy.init_node('tiny_lidar_node')
    print("Tiny Lidar Node Initialized")
    
    # Start tester
    try:
        TinyLidar()
    except rospy.ROSInterruptException:
        pass

