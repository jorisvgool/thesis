#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from gazebo_msgs.msg import ModelStates
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from tf.transformations import euler_from_quaternion, quaternion_matrix

class StereoDepthEstimator:
    def __init__(self):
        rospy.init_node('stereo_depth_estimator', anonymous=True)

        self.bridge = CvBridge()
        self.last_run_time = 0.0
        self.min_interval = 0.5  # seconds between frames

        self.left_img = None
        self.right_img = None
        self.pose1 = None
        self.pose2 = None

        # Image dimensions and intrinsics
        self.width = 800
        self.height = 600
        self.fov_x = 1.57  # Horizontal FOV in radians

        self.f_x = (self.width / 2) / np.tan(self.fov_x / 2)
        self.f_y = self.f_x  # Assuming square pixels

        self.c_x = self.width / 2
        self.c_y = self.height / 2

        self.camera_matrix = np.array([
            [self.f_x, 0,       self.c_x],
            [0,       self.f_y, self.c_y],
            [0,       0,        1]
        ])

        self.dist_coeffs = np.zeros(5)  # Assuming no lens distortion

        # ROS Subscribers and Publisher
        self.image_sub1 = rospy.Subscriber('/iris1_custom/iris_camera/camera/image_raw', Image, self.left_image_cb)
        self.image_sub2 = rospy.Subscriber('/iris2_custom/iris_camera/camera/image_raw', Image, self.right_image_cb)
        self.pose_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_cb)

        self.depth_pub = rospy.Publisher('/stereo_depth_image', Image, queue_size=1)

        # Stereo matcher
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=128,
            blockSize=3,
            P1=8 * 3 * 3 ** 2,
            P2=32 * 3 * 3 ** 2,
            disp12MaxDiff=1,
            uniquenessRatio=15,
            speckleWindowSize=50,
            speckleRange=2,
            preFilterCap=63,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )

    def get_position(self, pose):
        return np.array([pose.position.x, pose.position.y, pose.position.z])

    def get_rotation_matrix(self, orientation):
        quat = [orientation.x, orientation.y, orientation.z, orientation.w]
        rot_matrix = quaternion_matrix(quat)[:3, :3]
        return rot_matrix

    def model_cb(self, msg):
        try:
            idx1 = msg.name.index("iris1_custom")
            idx2 = msg.name.index("iris2_custom")
            self.pose1 = msg.pose[idx1]
            self.pose2 = msg.pose[idx2]
            self.try_process()
        except ValueError:
            rospy.logwarn_throttle(5, "iris1_custom or iris2_custom not found in /gazebo/model_states")

    def left_image_cb(self, msg):
        self.left_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.try_process()

    def right_image_cb(self, msg):
        self.right_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.try_process()

    def try_process(self):
        current_time = time.time()
        if current_time - self.last_run_time < self.min_interval:
            return

        if self.left_img is None or self.right_img is None:
            return

        if self.pose1 is None or self.pose2 is None:
            return

        self.last_run_time = current_time
        self.process_stereo()

    def process_stereo(self):
        try:
            left_gray = cv2.cvtColor(self.left_img, cv2.COLOR_BGR2GRAY)
            right_gray = cv2.cvtColor(self.right_img, cv2.COLOR_BGR2GRAY)

            pos1 = self.get_position(self.pose1)
            pos2 = self.get_position(self.pose2)
            rot1 = self.get_rotation_matrix(self.pose1.orientation)
            rot2 = self.get_rotation_matrix(self.pose2.orientation)

            # Relative transformation from camera 1 to camera 2
            R = rot2 @ rot1.T
            T = (pos2 - pos1).reshape(3, 1)

            baseline = np.linalg.norm(T)
            if baseline < 0.05:
                rospy.logwarn_throttle(5, f"Baseline too small: {baseline:.3f} m")
                return

            # Stereo rectification
            R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
                self.camera_matrix, self.dist_coeffs,
                self.camera_matrix, self.dist_coeffs,
                (self.width, self.height), R, T,
                flags=cv2.CALIB_ZERO_DISPARITY, alpha=0
            )

            map1x, map1y = cv2.initUndistortRectifyMap(
                self.camera_matrix, self.dist_coeffs, R1, P1, (self.width, self.height), cv2.CV_32FC1)
            map2x, map2y = cv2.initUndistortRectifyMap(
                self.camera_matrix, self.dist_coeffs, R2, P2, (self.width, self.height), cv2.CV_32FC1)

            rect_left = cv2.remap(left_gray, map1x, map1y, cv2.INTER_LINEAR)
            rect_right = cv2.remap(right_gray, map2x, map2y, cv2.INTER_LINEAR)

            # Compute disparity
            disparity = self.stereo.compute(rect_left, rect_right).astype(np.float32) / 16.0
            disparity[disparity <= 0.0] = 0.1  # prevent divide by zero

            # Depth map from disparity using known fx and baseline
            depth_map = (self.f_x * baseline) / disparity
            depth_map = cv2.medianBlur(depth_map.astype(np.float32), 5)
            kernel = np.ones((5, 5), np.uint8)
            depth_map = cv2.morphologyEx(depth_map, cv2.MORPH_CLOSE, kernel)

            # Normalize + colorize
            depth_colored = cv2.applyColorMap(depth_map.astype(np.uint8), cv2.COLORMAP_JET)

            # Publish
            depth_msg = self.bridge.cv2_to_imgmsg(depth_colored, encoding='bgr8')
            self.depth_pub.publish(depth_msg)

            # Debug
            # cv2.imshow("Depth", depth_colored)
            # cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"Stereo processing error: {e}")

if __name__ == '__main__':
    try:
        StereoDepthEstimator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
