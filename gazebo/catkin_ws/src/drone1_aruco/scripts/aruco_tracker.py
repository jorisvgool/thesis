#!/usr/bin/env python3

# Aruco marker detection code -- version 1.3.0
# Kalman filter + outlier detection
# Last edited: 28/03/2025
# Author: Joris van Gool

print("#######################################################")
print("#    Aruco marker detection code -- version 1.3.0     #")
print("#    Kalman filter + outlier detection                #")
print("#    Last edited: 28/03/2025                          #")
print("#    Author: Joris van Gool                           #")
print("#######################################################")

# Import packages
import cv2
import numpy as np
from cv2 import aruco
from collections import deque
import time
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()
latest_frame = None

def image_callback(msg):
    global latest_frame
    try:
        latest_frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except Exception as e:
        rospy.logerr(f"cv_bridge error: {e}")

# Subscribe to Gazebo camera feed
rospy.Subscriber("/iris/iris_camera/camera/image_raw", Image, image_callback)

# Camera intrinsics
width = 640
height = 480
fov_x = 1.047  # horizontal_fov in radians

f_x = (width / 2) / np.tan(fov_x / 2)
f_y = f_x  # Assume square pixels

c_x = width / 2
c_y = height / 2

camera_matrix = np.array([
    [f_x,   0,   c_x],
    [  0,  f_y,  c_y],
    [  0,   0,    1 ]
])

distortion_coeffs = np.zeros((1, 5))  # No distortion

# Define marker parameters
marker_size = 500  # [mm]
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
aruco_params = aruco.DetectorParameters_create()

# Initialize storage
n = 10  # moving window
angle_history = {}
distance_history = {}

# Initialize ROS node and publisher
rospy.init_node('aruco_publisher_v1_2', anonymous=True)
pub = rospy.Publisher('aruco_data', String, queue_size=10)
rate = rospy.Rate(100)  # 100 Hz

# Local functions
def add2buffer(history, marker_id, value):
    """Add new value to the rolling window"""
    if marker_id not in history:
        history[marker_id] = deque(maxlen=n)
    history[marker_id].append(value)

def is_outlier(value, history):
    """Detect whether a value is outlier """
    if len(history) < 5:
        return False
    mean = np.mean(history)
    std = np.std(history)
    return abs(value - mean) > 2 * std  # outlier iff >2*std

# Kalman class
class KalmanFilter1D:
    """Simple 1D Kalman filter with a constant-velocity model. Measure only the 'position' (angle / distance)."""
    def __init__(self, dt=0.01):
        # Model parameters
        self.dt = dt
        self.x = np.array([0.0, 0.0])
        self.P = np.eye(2)
        
        # Transition (F) and measurement (H) matrix
        self.F = np.array([[1.0, self.dt],[0.0, 1.0]])
        self.H = np.array([[1.0, 0.0]])
        
        # Process (Q) and measurement (R) noise covariance
        # (smaller Q => smoother output, but possibly more lag)
        # (larger R => trust the measurement less => smoother)
        self.Q = np.array([[1e-5, 0.0],[0.0, 1e-5]])
        self.R = np.array([[1e-1]])
    
    def predict(self):
        """Predict the next state based on model."""

        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
    
    def update(self, z):
        """Update the state estimate."""

        # Apply kalman's filter
        y = z - (self.H @ self.x)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        I = np.eye(2)
        self.P = (I - K @ self.H) @ self.P

# Initalize K storage
angle_kalman = {}
distance_kalman = {}

def kalman_estimate(kf_dict, marker_id, measurement, dt):
    """ Retrieve Kalman filter for marker_id."""

    # Marker defined yet?
    if marker_id not in kf_dict:
        kf_dict[marker_id] = KalmanFilter1D(dt=dt)
    else:
        # Update dt
        kf_dict[marker_id].dt = dt

    # Kalman predict
    kf_dict[marker_id].predict()
    # Kalman update
    kf_dict[marker_id].update(measurement)

    return kf_dict[marker_id].x[0]


# Time tracking
prev_time = time.time()

# Create window
cv2.namedWindow("ArUco Marker Detection", cv2.WINDOW_NORMAL)

# Main loop
while not rospy.is_shutdown():
    # Read frame
    if latest_frame is None:
       continue
    frame = latest_frame.copy()

    # Calculate dt from system clock
    current_time = time.time()
    dt = current_time - prev_time
    prev_time = current_time

    # Undistort image
    height, width = frame.shape[:2]
    new_cam_matrix, _ = cv2.getOptimalNewCameraMatrix(
        camera_matrix, distortion_coeffs, (width, height), 1, (width, height)
    )
    im_undistorted = cv2.undistort(
        frame, camera_matrix, distortion_coeffs, None, new_cam_matrix
    )

    # Convert image to grayscale for ArUco detection
    im_gray = cv2.cvtColor(im_undistorted, cv2.COLOR_BGR2GRAY)

    # Detect markers
    corners, ids, _ = aruco.detectMarkers(im_gray, aruco_dict, parameters=aruco_params)

    # If detected
    if ids is not None:
        # Estimate pose
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners, marker_size, camera_matrix, distortion_coeffs
        )

        for i in range(len(ids)):
            marker_id = int(ids[i][0])
            rvec = rvecs[i]
            tvec = tvecs[i]

            # Get rotation matrix
            rotation_matrix, _ = cv2.Rodrigues(rvec)

            # Calculate angle (pitch)
            angle_rad = np.arctan2(
                -rotation_matrix[2, 0],
                np.sqrt(rotation_matrix[2, 1]**2 + rotation_matrix[2, 2]**2)
            )
            angle_deg = np.degrees(angle_rad)

            # Calculate distance [mm]
            distance = np.linalg.norm(tvec)

            # Add to history buffers for outlier detection
            add2buffer(angle_history, marker_id, angle_deg)
            add2buffer(distance_history, marker_id, distance)

            # Skip if outlier
            if (is_outlier(angle_deg, angle_history[marker_id]) or
                is_outlier(distance, distance_history[marker_id])):
                continue

            # Kalman-filtered angle
            smoothed_angle = kalman_estimate(angle_kalman, marker_id, angle_deg, dt)
            # Kalman-filtered distance
            smoothed_distance = kalman_estimate(distance_kalman, marker_id, distance, dt)

            # Calculate center for console/debug
            center = tuple(np.mean(corners[i][0], axis=0).astype(int))

            # Console output
            output_str = f"{marker_id},{smoothed_distance:.2f},{smoothed_angle:.2f},{center[0]},{center[1]}"
            print(output_str)

            # Publish via ROS
            pub.publish(output_str)

            # Draw detected marker axes
            cv2.drawFrameAxes(im_undistorted, camera_matrix, distortion_coeffs, rvec, tvec, marker_size / 2)

            # Overlay text with angle and distance
            center = tuple(np.mean(corners[i][0], axis=0).astype(int))
            cv2.putText(
                im_undistorted,
                f"{smoothed_distance / 10:.2f} cm, {smoothed_angle:.1f}°",
                center,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

            cv2.imshow("ArUco Marker Detection", im_undistorted)
            cv2.waitKey(1)  # Keeps the window responsive


# Clean up
cv2.destroyAllWindows()
