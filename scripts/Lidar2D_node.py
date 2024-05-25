#!/usr/bin/env python3

"""
Simple 2D Lidar Node that publishes a smoothed laser scan and the closest detected object inside the FOV.

Devson Butani - 2024
"""

from math import pi, radians

import numpy as np
import rospy
from actor_lidar.cfg import Lidar2DConfig
from dynamic_reconfigure.server import Server
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64


class Lidar2DProcessor:
    def __init__(self):
        """Initialize the 2D Lidar Node"""

        rospy.init_node("lidar_2d_node", anonymous=True)
        rospy.loginfo("2D Lidar Node Initialized")

        # Dynamic reconfiguration server
        Server(Lidar2DConfig, self.dynamic_reconfigure_callback)
        self.dyn_rcfg_initialized = False

        # Waiting for dynamic reconfigure to spin up
        while not rospy.is_shutdown() and not self.dyn_rcfg_initialized:
            rospy.logwarn("Waiting for dynamic reconfigure to spin up...")
            rospy.sleep(0.1)

        # Initialize variables
        self.closest_point = -1.0
        self.output_smoothed_scan = rospy.get_param("output_smoothed_scan", False)

        # Subscriber to receive laser scan data
        rospy.Subscriber("scan", LaserScan, self.laser_callback, queue_size=1)

        # Publisher to publish the closest detected object
        self.closest_object_output = rospy.Publisher("closest_object", Float64, queue_size=1)

        if self.output_smoothed_scan:
            # Publisher to publish smoothed laser scan data
            self.smoothed_scan_output = rospy.Publisher("smoothed_scan", LaserScan, queue_size=1)

        # Spin the node
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.loginfo("actor_control node shutting down.")

    def dynamic_reconfigure_callback(self, config, level):
        """Dynamic Reconfigure Callback"""

        self.dyn_rcfg_initialized = True
        self.config_ = config
        return config

    def laser_callback(self, laser_scan_msg):
        """Process Laser Scan Data using numpy"""

        # Convert laser scan data to numpy array
        scan = np.array(laser_scan_msg.ranges)

        # Filter laser scan data for boundary cases using numpy
        scan[np.isnan(scan)] = self.config_.max_range  # Replace NaN values with max_range
        scan[np.isinf(scan)] = self.config_.max_range  # Replace Inf values with max_range
        scan[scan < self.config_.min_range] = self.config_.max_range  # Replace values below min_range with max_range

        # Publish smoothed laser scan data
        if self.output_smoothed_scan:
            laser_scan_msg.ranges = list(scan)  # Use the input message as-is to retain all metadata for each message
            self.smoothed_scan_output.publish(laser_scan_msg)

        # Narrow the Field of View (FOV) using center_offset and field_of_view
        # NOTE: Math is in radians and array indices
        total_angles = scan.shape[0]
        center_index = round(
            (radians(self.config_.center_offset) - laser_scan_msg.angle_min) / laser_scan_msg.angle_increment
        )
        half_fov_indices = round(radians(self.config_.field_of_view) / (2 * laser_scan_msg.angle_increment))
        start_index = max(center_index - half_fov_indices, 0)
        end_index = min(center_index + half_fov_indices, total_angles)
        fov_scan = scan[start_index:end_index]

        # Find the closest object in the FOV
        self.closest_point = np.min(fov_scan) if fov_scan.shape[0] > 0 else self.config_.max_range

        # Publish the closest object
        closest_point_msg = Float64()
        closest_point_msg.data = self.closest_point
        self.closest_object_output.publish(closest_point_msg)


if __name__ == "__main__":
    # Initialize ROS node
    Lidar2DProcessor()
