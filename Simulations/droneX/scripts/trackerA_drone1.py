#!/usr/bin/env python3

# ArUco marker detection code -- version 1.4.0
# Last edited: 11/06/2025
# Author: Joris van Gool

print("#######################################################")
print("#    ArUco marker detection code -- version 1.4.0     #")
print("#    Last edited: 11/06/2025                          #")
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

simulation = True

def image_callback(msg):
    global latest_frame
    try:
        latest_frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except Exception as e:
        rospy.logerr(f"cv_bridge error: {e}")

# Subscribe to Gazebo camera feed
rospy.Subscriber("/iris1_custom/iris_camera/camera/image_raw", Image, image_callback)

# Camera intrinsics
width = 800
height = 600
fov_x = 1.57 # horizontal_fov in radians

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
theta_kalman = {}
beta_kalman = {}
zd_kalman = {}

# Moving windows
theta_history = {}
beta_history = {}
zd_history = {}

# Initialize ROS node and publisher
rospy.init_node('aruco_node_drone1', anonymous=True)
pub = rospy.Publisher('/drone1/aruco_data', String, queue_size=10)
rate = rospy.Rate(100) # 20 Hz

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
        self.Q = np.array([[1e-4, 0.0],[0.0, 1e-2]])
        self.R = np.array([[1e-4]])  # Measurement noise (e.g., angle noise)
    
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
prev_time = rospy.Time.now() if simulation else time.time()

# Main loop
while not rospy.is_shutdown():
    # Read frame
    if latest_frame is None:
       continue
    frame = latest_frame.copy()

    # Calculate & update dt for PID
    if simulation:
        current_time = rospy.Time.now()
        dt = (current_time - prev_time).to_sec()
        prev_time = current_time
    else:
        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time
    dt = max(dt, 1e-6)

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

            # Calculate distance [mm]
            zd = np.linalg.norm(tvec)  # Euclidean distance

            # Calculate horizontal angle to marker
            # phi = np.degrees(np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0]))
            # beta = np.degrees(np.arctan2(tvec[0], tvec[2]))
            # theta = (beta - phi + 180) % 360 - 180  # Normalize to [-180, 180]
            # theta = np.clip(theta, -90, 90)

            # print(f"Phi: {phi:.2f}°, Beta: {beta:.2f}°, Theta: {theta:.2f}°")

            tvec_cam = tvec[0]  # marker position in camera frame

            # Invert pose to get camera position in marker frame:
            # If R and t transform marker to camera frame,
            # then camera_in_marker = -R.T @ t
            R = rotation_matrix.T  # transpose to invert rotation
            t_marker_to_cam = -R @ tvec_cam  # drone pos in marker frame

            # The marker's local Z axis is its "forward" direction
            # So we now compute the angle between:
            # - Z-axis of marker = [0, 0, 1]
            # - vector to drone (in marker frame): t_marker_to_cam

            # Project both into XY plane (horizontal plane in marker frame)
            drone_vec_xy = t_marker_to_cam[:2]
            drone_vec_xy /= np.linalg.norm(drone_vec_xy)

            marker_fwd_xy = np.array([0, 1])  # Marker forward = positive Y in its own frame

            # Signed angle between marker_fwd and drone vector in XY plane
            dot = np.dot(marker_fwd_xy, drone_vec_xy)
            cross = marker_fwd_xy[0]*drone_vec_xy[1] - marker_fwd_xy[1]*drone_vec_xy[0]
            angle_rad = np.arctan2(cross, dot)
            angle_deg = np.degrees(angle_rad)

            # Compute angle in XY plane (usually X: right, Y: down, Z: forward in camera frame)
            # Get marker's x-axis vector in camera frame
            marker_x_cam = rotation_matrix[:, 0]

            # We only care about the XY plane (ignore Z)
            dx = t_marker_to_cam[0]
            dy = t_marker_to_cam[1]

            # Angle in radians
            beta_rad = -np.arctan2(dx, dy)

            # Convert to degrees if you want
            beta_deg = np.degrees(beta_rad)
            # print(f"Beta (rad): {beta_deg:.2f}")

            # print(f"Theta: {angle_deg:.2f}°")

  

            # Add to history buffers for outlier detection
            add2buffer(theta_history, marker_id, angle_deg)
            add2buffer(zd_history, marker_id, zd)
            add2buffer(beta_history, marker_id, beta_deg)

            # Skip if outlier
            if (is_outlier(angle_deg, theta_history[marker_id]) or
                is_outlier(zd, zd_history[marker_id])) or is_outlier(beta_deg, beta_history[marker_id]):
                continue

            # Kalman-filtering
            theta_K = kalman_estimate(theta_kalman, marker_id, angle_deg, dt)
            zd_K = kalman_estimate(zd_kalman, marker_id, zd, dt)
            beta_k = kalman_estimate(beta_kalman, marker_id, beta_deg, dt)
            # print(f"Beta: {beta_k:.2f}°")

            # Calculate center for console/debug
            center = tuple(np.mean(corners[i][0], axis=0).astype(int))

            # Console output
            output_str = f"{marker_id},{theta_K:.2f},{beta_k:.2f},{zd_K:.2f},{center[0]},{center[1]}"

            # Publish via ROS
            pub.publish(output_str)

            # Small pause
            # time.sleep(0.01)

# Clean up
cv2.destroyAllWindows()
