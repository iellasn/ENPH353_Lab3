#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

import numpy as np
import cv2 as cv

# Global constants (optional, but we'll use them as defaults)
THRESHOLD_VAL = 125
CIRCLE_RADIUS = 30
KERNEL_SIZE = 5
SCAN_HEIGHT = 799
"""
@brief A class for detecting and tracking lines from the ros topic capturing videos of the path 
       and adjusting vehicle speed so that it follows the line.
@details This class allows the user to process camera input of a robot driving on a path and 
         adjust steering to keep it centered on the line.

@author 
@date 

800 x 800 image numpy array
"""

class LineFollower:
    def __init__(self):
        # 1. Initialize ROS node
        rospy.init_node('path_following', anonymous=True)

        # 2. Publisher for velocity commands
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # 3. Subscriber for camera images
        self.sub = rospy.Subscriber('/camera/image_raw', Image, self.callback)

        # 4. Create CvBridge for image conversion
        self.bridge = CvBridge()

        # 5. Create a Twist object for reuse
        self.move = Twist()

        self.threshold = THRESHOLD_VAL
        self.kernel_size = KERNEL_SIZE
        self.scan_height = SCAN_HEIGHT

        self.rate = rospy.Rate(2)

        # cv.namedWindow("before process", cv.WINDOW_NORMAL)
        # cv.namedWindow("after process", cv.WINDOW_NORMAL)

    def callback(self, image_msg):
        """
        Callback function for each new camera frame.
        """
        try:
            # Convert ROS Image to OpenCV image
            cv_img = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")
            return

        # 1. Preprocess the image (blur, threshold, etc.)
        pre_processed_img = self.pre_processing_frame(cv_img)

        # 2. Find the line's midpoint in the preprocessed image
        middle_pt = self.finding_point(pre_processed_img)
        # cv.circle(pre_processed_img, (middle_pt, self.scan_height), self.dot_radius,(0,0,255), -1)
        # cv.imshow(pre_processed_img)

        if middle_pt == -1:
            self.move.linear.x = 0
            self.move.angular.z = 0.2
        else:
            # 3. Compute error: difference between image center and line midpoint
            #    If the line is at the exact center, error = 0
            img_width = cv_img.shape[1] #number of columns
            img_center = img_width / 2.0 # center x cord
            error = img_center - middle_pt
            print(f"error:{error}")

            # 4. Set the robot's forward (linear) and angular velocity
            self.move.linear.x = 0.2             # Forward speed
            kp = 0.033                        # Proportional gain
            self.move.angular.z = kp * error

        # 5. Publish the Twist command
        try:
            self.pub.publish(self.move)
            # print(f"publishin command {self.move}")
        except Exception as e:
            print(f"failed to publish command {e}")

    def pre_processing_frame(self, frame):
        """
        Convert the frame to grayscale, blur it, and apply a binary threshold
        to isolae the darker line.
        """

        # print("starting pre processing")

        try:
            # cv.imshow("before process", frame)
            # cv.waitKey(1)

            # Convert to grayscale
            gray_scaled = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            # print("converted to grayscale")

            # Gaussian blur
            gaussian_blurred = cv.GaussianBlur(
                gray_scaled, (self.kernel_size, self.kernel_size), 0
            )
            # print("applied gaussian blur")

            # Apply threshold
            # If the line is dark, use THRESH_BINARY_INV to make it white
            _, binary_threshold = cv.threshold(
                gaussian_blurred, self.threshold, 255, cv.THRESH_BINARY
            ) #inverse binary is if pixel intesity > threshold it is made 0 (black)

            # print("applied threshold")
            # cv.imshow("before process", frame)
            # cv.imshow("after process", binary_threshold)
            # cv.waitKey(1)
            # print("displayed after process frame")

            return binary_threshold
        except Exception as e:
            print(f"error in preprocessing {e}")
    
    # def finding_point(self, pre_processed_frame):
    #     """
    #     Find the horizont
    #     al center of the line by scanning multiple rows and averaging
    #     the detected line centers.
    #     """
    #     try:
    #         kernal = np.ones((5,5), np.uint8)
    #         dilated = cv.dilate(pre_processed_frame, kernal, iterations=1)

    #         self.scan_height = dilated.shape[0] - 1
    #         scan_line  = dilated[self.scan_height, :]

    #         line_start = -1
    #         line_width = 0
    #         in_line = False 

    #         for x in range(len(scan_line)):
    #             if scan_line[x] < self.threshold:
    #                 if not in_line:
    #                     line_start = x
    #                     in_line = True
    #                 line_width += 1
    #             elif in_line:
    #                 break
            
    #         if line_start >= 0:
    #             center_x_cord = line_start + (line_width // 2)
    #             return center_x_cord
    #         else:
    #             return -1

            
        # except Exception as e:
        #     print(f"Error in finding_point: {e}")
        #     return pre_processed_frame.shape[1] // 2
    
    def finding_point(self, pre_processed_frame):
        """
        Find the horizontal center of the line by scanning one row at self.scan_height.
        We find edges in that row and pick the midpoint between the first and last edge.
        """
        # Validate scan_height just in case
        if self.scan_height >= pre_processed_frame.shape[0]:
            print(f"Scan height: {self.scan_height},image has {pre_processed_frame.shape[0]} rows")
            # If scan_height is out of range, default to the center
            self.scan_height = pre_processed_frame[0] - 1
            #.shape returns a tuple (row, column), we are accessing the column here

        # Extract a single row from the image
        scan_line = pre_processed_frame[self.scan_height, :]

        # Find where pixel values change (edges)
        edges = np.where(np.diff(scan_line) != 0)[0]

        # If we found at least two edges, we can define a left and right edge
        if len(edges) >= 2:
            left_edge = edges[0]
            right_edge = edges[-1]
            center_x_cord = (left_edge + right_edge) // 2
        else:
            # If we can't find edges, just default to image center
            center_x_cord = pre_processed_frame.shape[1] // 2

        print(center_x_cord)
        return center_x_cord

    
    def run(self):
        """
        Main loop if you want to do something periodically.
        If you only rely on the callback, you can simply use rospy.spin().
        """
        while not rospy.is_shutdown():
            # Your callback runs on every incoming image, so we can just sleep here.
            self.rate.sleep()


if __name__ == "__main__":
    line_follower = LineFollower()
    # You can call run() if you want a loop + the callback
    line_follower.run()
    # Or you could just do:
    # rospy.spin()
    
