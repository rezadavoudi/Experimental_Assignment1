#!/usr/bin/env python3

# ROS libraries
import roslib
import rospy
import time
import math

# Python libs
import sys

# Numpy and Scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2
import imutils

# ROS Messages
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import CompressedImage, CameraInfo

class CameraFixController:
    def __init__(self):
        rospy.init_node('fix_controller')

        self.CameraCenter = Point()
        self.MarkerCenter = Point()
        self.marker_id_list = []  # List to hold detected marker IDs
        self.marker_centers_dict = {}  # Dictionary for marker centers
        self.Id_number = 0
        self.Info_gathering_mode = True  # Flag to control data gathering
        self.Reached = False
        self.Current_marker = 0

        # Publishers
        self.image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage, queue_size=1)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Subscribers
        rospy.Subscriber("/robot/camera1/image_raw/compressed", CompressedImage, self.controller_callback, queue_size=1)
        rospy.Subscriber("/robot/camera1/camera_info", CameraInfo, self.camera_callback, queue_size=1)
        rospy.Subscriber("/marker/id_number", Int32, self.id_callback, queue_size=1)
        rospy.Subscriber("/marker/center_loc", Point, self.center_callback, queue_size=1)

    # Callback to find the center of the camera
    def camera_callback(self, msg):
        self.CameraCenter.x = msg.height / 2
        self.CameraCenter.y = msg.width / 2

    # Callback to update the ID number of the marker
    def id_callback(self, msg):
        self.Id_number = msg.data

    # Callback to update the center of the marker
    def center_callback(self, msg):
        self.MarkerCenter.x = msg.x
        self.MarkerCenter.y = msg.y

        if self.Id_number and self.Id_number not in self.marker_id_list:
            self.marker_centers_dict[self.Id_number] = (msg.x, msg.y)

    # Main controller callback
    def controller_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        vel = Twist()

        if self.Info_gathering_mode:
            if len(self.marker_id_list) < 7:
                vel.linear.x = 0
                vel.angular.z = 0.7

                if self.Id_number and self.Id_number not in self.marker_id_list:
                    self.marker_id_list.append(self.Id_number)
                    self.marker_id_list.sort()
                    rospy.loginfo(f"Updated marker IDs: {self.marker_id_list}")
            else:
                self.Info_gathering_mode = False
                rospy.loginfo("Finished gathering all markers. Starting drawing phase.")
        else:
            if self.marker_id_list:
                self.Current_marker = self.marker_id_list[0]
                target_x = self.MarkerCenter.x
                target_y = self.MarkerCenter.y

                if abs(self.CameraCenter.x - target_x) < 10 and self.Id_number == self.Current_marker:
                    self.Reached = True
                    vel.angular.z = 0
                    rospy.loginfo(f"Reached marker {self.Current_marker}")

                    # Draw a circle around the marker position on the image
                    cv2.circle(image_np, (int(target_x), int(target_y)), 20, (0, 255, 0), 3)

                    image_msg = CompressedImage()
                    image_msg.header.stamp = rospy.Time.now()
                    image_msg.format = "jpeg"
                    image_msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tobytes()

                    self.image_pub.publish(image_msg)

                if self.Reached:
                    self.marker_id_list.pop(0)
                    self.Reached = False
                    vel.angular.z = 0
                elif self.CameraCenter.x > target_x and self.Id_number == self.Current_marker:
                    vel.angular.z = 0.3
                    rospy.loginfo("Turn left")
                elif self.CameraCenter.x < target_x and self.Id_number == self.Current_marker:
                    vel.angular.z = -0.3
                    rospy.loginfo("Turn right")
                else:
                    vel.angular.z = 0.5
                    rospy.loginfo("Keep searching")
            else:
                vel.angular.z = 0
                rospy.loginfo("All markers found")

        self.velocity_publisher.publish(vel)

def main():
    CameraFixController()
    rospy.spin()

if __name__ == '__main__':
    main()
