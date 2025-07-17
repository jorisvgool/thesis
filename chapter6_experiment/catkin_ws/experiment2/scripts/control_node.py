#!/usr/bin/env python3

# Formation control node Kalman (sine)
# Last edited: 26/06/2025
# Author: Joris van Gool

print("#######################################################")
print("#    Formation control node Kalman (sine)             #")
print("#    Last edited: 26/06/2025                          #")
print("#    Author: Joris van Gool                           #")
print("#######################################################")

# Imports
import rospy
import numpy as np
import time
import atexit

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import TwistStamped, PoseStamped, Vector3Stamped
import sys
sys.path.append('/home/joris/catkin_ws/src/experiment1/scripts')

import custom_functions as cf
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist
from marvelmind_nav.msg import hedge_pos_ang

# Kalman filter function
def kalman_filter(x_est, P_est, z, dt, t):
    # Extract estimated frequencies
    omega1 = x_est[14]
    omega2 = x_est[15]

    # Trigonometric terms
    sin1 = np.sin(omega1 * t)
    cos1 = np.cos(omega1 * t)
    sin2 = np.sin(omega2 * t)
    cos2 = np.cos(omega2 * t)

    # Initialize transition matrix
    A = np.eye(16)

    # Position update from velocity model v(t)
    A[0, 2] = dt
    A[1, 3] = dt

    # Velocity update from polynomial + harmonic model
    A[2, 2] = 0
    A[2, 4] = 1
    A[2, 6] = t
    A[2, 8] = t**2
    A[2,10] = sin1
    A[2,12] = cos1
    A[2,14] = t * (x_est[10]*cos1 - x_est[12]*sin1)

    A[3, 3] = 0
    A[3, 5] = 1
    A[3, 7] = t
    A[3, 9] = t**2
    A[3,11] = sin2
    A[3,13] = cos2
    A[3,15] = t * (x_est[11]*cos2 - x_est[13]*sin2)

    # Observation matrix
    H = np.zeros((2, 16))
    H[0, 0] = 1
    H[1, 1] = 1

    # Process noise matrix
    Q = np.diag([
        1e-8, 1e-8,    # position
        1e-5, 1e-5,    # velocity
        1e-1, 1e-1,    # a0
        1e-1, 1e-1,    # a1
        1e-1, 1e-1,    # a2
        1e-5, 1e-5,    # alpha
        1e-5, 1e-5,    # beta
        1e-1, 1e-1     # omega
    ])

    # Observation noise matrix
    R = 1e-3*np.eye(2)

    # EKF Prediction
    x_pred = A @ x_est
    P_pred = A @ P_est @ A.T + Q

    # Kalman Gain
    S = H @ P_pred @ H.T + R
    K = P_pred @ H.T @ np.linalg.inv(S)

    # Update step
    x_est = x_pred + K @ (z - H @ x_pred)
    P_est = (np.eye(16) - K @ H) @ P_pred

    return x_est, P_est

# Global callbacks
current_state = State()
def state_cb(msg):
    global current_state
    current_state = msg

current_pose = None
def marvelmind_cb(msg):
    global current_pose
    current_pose = msg

# Main function
def main():

    # Simulation & kalman active?
    kalman = True

    # Logbook
    log_data = [] 
    header = ["time","e1","e2","e3","z1","z2","z3","x","y","h"]
    atexit.register(lambda: cf.save_log(log_data, cf.log_path(), header))

    # State machine
    TAKEOFF = 0
    FORMATION = 1
    control_state = TAKEOFF

    # ROS node init
    rospy.init_node("control_node_1", anonymous=True)

    # ROS subscribers
    rospy.Subscriber("/mavros/state", State, state_cb)
    rospy.Subscriber("/hedge_pos_ang", hedge_pos_ang, marvelmind_cb)

    # ROS publishers
    vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
    error_pub = rospy.Publisher('error_vector', Vector3Stamped, queue_size=10)

    # ROS services
    rospy.wait_for_service("/mavros/cmd/arming")
    rospy.wait_for_service("/mavros/set_mode")
    arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

    # -----------------------------------------------
    # Controller configuration
    # -----------------------------------------------

    # General parameters
    l = 1           # distance potential exponent
    vLIM = 5        # velocity limit (XY)
    c1 = 0.33       # controller gain

    # Altitute control
    h_star = 1.5      # desired altitute
    vLIMh = 2       # velocity limit (Z)
    z_pid = cf.PID(Kp=1.5, Ki=0.0, Kd=0.5, dt=0.01, output_limits=(-vLIMh, vLIMh))

    # Start position drone
    t = np.array([current_pose.x_m,current_pose.y_m,current_pose.z_m])

    # Incidence matrix
    B = np.array([
        [1, 0, -1],
        [-1, 1, 0],
        [0, -1, 1]
    ])
    B_bar = np.kron(B, np.eye(2))

    # Desired edge lengths
    d = np.array([np.sqrt(4), np.sqrt(4), np.sqrt(4)])

    # Reference fixed points (anchors)
    a = np.array([2.0,0.0])
    b = np.array([1.0,np.sqrt(3)])
    b0 = b

    # Takeoff control
    takeoff_goal = [0.0, 0.0, h_star]
    pos_tolerance = 0.4

    # -----------------------------------------------
    # -----------------------------------------------

    # ROS rate & timing
    rate = rospy.Rate(20)
    prev_time = time.time()

    # Wait for FCU connetion
    while not rospy.is_shutdown() and not current_state.connected:
        rospy.loginfo_throttle(2, "[drone] Waiting for FCU connection...")
        rate.sleep()
    rospy.loginfo("[drone] FCU connected")

    # Take-off objective
    pose_cmd = PoseStamped()
    pose_cmd.pose.position.x = takeoff_goal[0]
    pose_cmd.pose.position.y = takeoff_goal[1]
    pose_cmd.pose.position.z = takeoff_goal[2]

    # Setpoints to enable OFFBOARD mode
    for _ in range(100):
        pos_pub.publish(pose_cmd)
        rate.sleep()
    last_req = rospy.Time.now()

    # Kalman set-up
    x_est = np.concatenate([np.array([a[0], a[1]]), np.zeros(12),np.array([0.21,0.21])])
    P_est = 1e-3*np.eye(16)

    # Control loop
    while not rospy.is_shutdown():

        # Check & request OFFBOARD mode
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req > rospy.Duration(5.0)):
            if set_mode_client(custom_mode="OFFBOARD").mode_sent:
                rospy.loginfo("[drone] OFFBOARD mode set")
            last_req = rospy.Time.now()

        # Check & request arming mode
        elif not current_state.armed and (rospy.Time.now() - last_req > rospy.Duration(5.0)):
            if arming_client(True).success:
                rospy.loginfo("[drone] Drone armed")
            last_req = rospy.Time.now()
            
        # CONTROL state
        if control_state == TAKEOFF:
            pos_pub.publish(pose_cmd)

            # Distance to goal
            dx = current_pose.x_m - takeoff_goal[0] - t[0]
            dy = current_pose.y_m - takeoff_goal[1] - t[1]
            dz = current_pose.z_m - takeoff_goal[2] - t[2]
            distance = np.linalg.norm([dx, dy, dz])

            # If goal take-off goal reached, switch state
            if distance < pos_tolerance:
                rospy.loginfo("[drone] Takeoff completed, switching to formation control...")
                control_state = FORMATION
                t_start = (rospy.Time.now()).to_sec()
                rospy.loginfo(f"Formation control started at t={t_start:.2f}s")

        # FORMATION state
        elif control_state == FORMATION:
            # Drone's coordinates
            x = current_pose.x_m - t[0]
            y = current_pose.y_m - t[1]
            h = current_pose.z_m - t[2]

            # Calculate & update dt for PID
            current_time = time.time()
            dt = current_time - prev_time
            prev_time = current_time
            ttime = current_time-t_start
            dt = max(dt, 1e-6)
            z_pid.dt = dt

            # Move anchors
            if ttime > 30 and ttime < 90:
                s = 1.5 * np.sin((2*np.pi/30) * (ttime-30))
            else:
                s = 0
            a = np.array([2.0,s])
            b = np.array([1.0,s+np.sqrt(3)])

            # Positions (abolute and relative)
            p = np.array([x,y,a[0],a[1],b[0],b[1]])
            z = B_bar.T @ p

            # Calculate control parameters
            z_norm = np.linalg.norm(z.reshape((3, 2)), axis=1)
            e = z_norm**l - d**l 
            z_tilde = z_norm**(l-2)
            Dz = cf.block_diag(z)
            Dz_tilde = np.diag(z_tilde)

            # Calculate control law
            p_dot = -c1 * B_bar @ Dz @ Dz_tilde @ e
            x_dot = np.clip(p_dot[0], -vLIM, vLIM)
            y_dot = np.clip(p_dot[1], -vLIM, vLIM)

            # Kalman
            if kalman:
                x_est, P_est = kalman_filter(x_est, P_est, b, dt, ttime)
                v = x_est[2:4]
                # print(v)
            else:
                v = np.zeros(2)

            # Publish errors
            ep = z_norm - d
            msg = Vector3Stamped()
            msg.header.stamp = rospy.Time.now()
            msg.vector.x = ep[0]
            msg.vector.y = ep[1]
            msg.vector.z = ep[2]
            error_pub.publish(msg)

            # Altitude control
            z_dot = z_pid.compute(h_star - h)

            # Publish velocity command
            twist = TwistStamped()
            twist.twist.linear.x = x_dot + v[0]
            twist.twist.linear.y = y_dot + v[1]
            twist.twist.linear.z = z_dot
            vel_pub.publish(twist)

            # Record data
            log_data.append((ttime, ep[0], ep[1], ep[2], z_norm[0], z_norm[1], z_norm[2],x, y, h))

        rate.sleep()

if __name__ == "__main__":
    main()
