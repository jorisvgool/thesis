#!/usr/bin/env python3

# Formation control node kalman
# Last edited: 03/07/2025
# Author: Joris van Gool

print("#######################################################")
print("#    Formation control node (kalman)                  #")
print("#    Last edited: 03/07/2025                          #")
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
import custom_functions as cf

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import numpy as np

# Kalman filter function
def kalman_filter(x_est, P_est, z, dt):
    # Define matrices
    Akf = np.array([
        [1, dt, dt**2 / 2],
        [0, 1, dt],
        [0, 0, 1]
    ])
    Akf_bar = np.kron(Akf, np.eye(2))

    H = np.array([
        [1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0]
    ])

    Q = np.diag([1e-4, 1e-4, 1e-2, 1e-2, 1e-1, 1e-1])
    R = 10 * np.eye(2)

    # Prediction step
    x_pred = Akf_bar @ x_est
    P_pred = Akf_bar @ P_est @ Akf_bar.T + Q

    # Kalman Gain
    S = H @ P_pred @ H.T + R
    K = P_pred @ H.T @ np.linalg.inv(S)

    # Update step
    x_est = x_pred + K @ (z - H @ x_pred)
    P_est = (np.eye(6) - K @ H) @ P_pred

    return x_est, P_est


# Global callbacks
current_state = State()
def state_cb(msg):
    global current_state
    current_state = msg

current_pose = None
def pose_cb(msg):
    global current_pose
    current_pose = msg

# Main function
def main():

    # Simulation & kalman active?
    simulation = True
    kalman = True

    # Logbook
    log_data = [] 
    header = ["time", "e1", "e2", "e3", "z1", "z2", "z3", "x", "y", "h"]
    atexit.register(lambda: cf.save_log(log_data, cf.log_path(), header))

    # State machine
    TAKEOFF = 0
    FORMATION = 1
    control_state = TAKEOFF

    # ROS node init
    rospy.init_node("control_node_1", anonymous=True)

    # ROS subscribers
    rospy.Subscriber("/drone1/mavros1/state", State, state_cb)
    rospy.Subscriber("/drone1/mavros1/local_position/pose", PoseStamped, pose_cb)

    # ROS publishers
    vel_pub = rospy.Publisher("/drone1/mavros1/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    pos_pub = rospy.Publisher("/drone1/mavros1/setpoint_position/local", PoseStamped, queue_size=10)
    error_pub = rospy.Publisher('error_vector', Vector3Stamped, queue_size=10)

    # ROS services
    rospy.wait_for_service("/drone1/mavros1/cmd/arming")
    rospy.wait_for_service("/drone1/mavros1/set_mode")
    arming_client = rospy.ServiceProxy("/drone1/mavros1/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("/drone1/mavros1/set_mode", SetMode)

    # -----------------------------------------------
    # Controller configuration
    # -----------------------------------------------

    # General parameters
    l = 2           # distance potential exponent
    vLIM = 1        # velocity limit (XY)
    c1 = 0.15       # controller gain

    # Altitute control
    h_star = 2      # desired altitute
    vLIMh = 1       # velocity limit (Z)
    z_pid = cf.PID(Kp=1.2, Ki=0.1, Kd=0.1, dt=0.01, output_limits=(-vLIMh, vLIMh))

    # Start position drone
    t = np.array([0.0,3.0,0.0])

    # Incidence matrix
    B = np.array([
        [1, 0, -1],
        [-1, 1, 0],
        [0, -1, 1]
    ])
    B_bar = np.kron(B, np.eye(2))

    # Desired edge lengths
    d = np.array([np.sqrt(9), np.sqrt(9), np.sqrt(9)])

    # Reference fixed points (anchors)
    a = np.array([0.0,0.0])
    b = np.array([2.598,1.5])
    b0 = np.array([2.598,1.5])


    # Takeoff control
    takeoff_goal = [0.0, 0.0, h_star]
    pos_tolerance = 0.1

    # -----------------------------------------------
    # -----------------------------------------------

    # ROS rate & timing
    rate = rospy.Rate(20)
    prev_time = rospy.Time.now().to_sec() if simulation else time.time()

    # Wait for FCU connetion
    while not rospy.is_shutdown() and not current_state.connected:
        rospy.loginfo_throttle(2, "[Drone1] Waiting for FCU connection...")
        rate.sleep()
    rospy.loginfo("[Drone1] FCU connected")

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
    x_est = np.concatenate([np.array([a[0], a[1]]), np.zeros(4)])
    P_est = 1e-3*np.eye(6)

    # Control loop
    while not rospy.is_shutdown():

        # Check & request OFFBOARD mode
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req > rospy.Duration(5.0)):
            if set_mode_client(custom_mode="OFFBOARD").mode_sent:
                rospy.loginfo("[Drone1] OFFBOARD mode set")
            last_req = rospy.Time.now()

        # Check & request arming mode
        elif not current_state.armed and (rospy.Time.now() - last_req > rospy.Duration(5.0)):
            if arming_client(True).success:
                rospy.loginfo("[Drone1] Drone armed")
            last_req = rospy.Time.now()
            
        # CONTROL state
        if control_state == TAKEOFF:
            pos_pub.publish(pose_cmd)

            # Distance to goal
            dx = current_pose.pose.position.x - takeoff_goal[0]
            dy = current_pose.pose.position.y - takeoff_goal[1]
            dz = current_pose.pose.position.z - takeoff_goal[2]
            distance = np.linalg.norm([dx, dy, dz])

            # If goal take-off goal reached, switch state
            if distance < pos_tolerance:
                rospy.loginfo("[Drone1] Takeoff completed, switching to formation control...")
                control_state = FORMATION
                t_start = (rospy.Time.now()).to_sec()
                rospy.loginfo(f"Formation control started at t={t_start:.2f}s")

        # FORMATION state
        elif control_state == FORMATION:
            # Drone's coordinates
            x = current_pose.pose.position.x + t[0]
            y = current_pose.pose.position.y + t[1]
            h = current_pose.pose.position.z + t[2]

            # Calculate & update dt for PID
            if simulation:
                current_time = rospy.Time.now().to_sec()
                dt = (current_time - prev_time)
                prev_time = current_time
            else:
                current_time = time.time()
                dt = current_time - prev_time
                prev_time = current_time
            ttime = current_time-t_start
            dt = max(dt, 1e-6)
            z_pid.dt = dt

            # Move anchors
            if ttime > 30:
                b[0] += 0.69*dt
                a = b - b0
            
            if ttime > 60:
                print("simulation ended")
                break

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

            # Kalman=
            if kalman:
                x_est, P_est = kalman_filter(x_est, P_est, b, dt)
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
