#!/usr/bin/env python3

# Control node for formation (base simulation)
# By: Joris van Gool
# Date: 15/05/2025

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


# Global state variables (set by callbacks)
current_state = State()
def state_cb(msg):
    global current_state
    current_state = msg

current_pose = None
def pose_cb(msg):
    global current_pose
    current_pose = msg

new_data = False
def aruco_cb(msg):
    global theta, zd, cx, new_data, beta
    try:
        parts = msg.data.split(',')
        theta = np.radians(float(parts[1]))  # degrees to radians
        beta = np.radians(float(parts[2]))
        zd = float(parts[3])/1000
        cx = float(parts[4])
        new_data = True

    except Exception as e:
        rospy.logwarn(f"Failed to parse /drone1/aruco_data: {e}")

# Main control logic
def main():

    # Simulation?
    simulation = True

    global new_data

    # Logbook set-up
    log_data = [] 
    atexit.register(lambda: cf.save_log(log_data, cf.log_path() ))

    # State machine
    TAKEOFF = 0
    FORMATION = 1
    control_state = TAKEOFF

    # ROS node init
    rospy.init_node("control_node_1", anonymous=True)

    # ROS subscribers
    rospy.Subscriber("/drone1/mavros1/state", State, state_cb)
    rospy.Subscriber("/drone1/mavros1/local_position/pose", PoseStamped, pose_cb)
    rospy.Subscriber("/drone1/aruco_data", String, aruco_cb)

    # ROS publishers
    vel_pub = rospy.Publisher("/drone1/mavros1/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    pos_pub = rospy.Publisher("/drone1/mavros1/setpoint_position/local", PoseStamped, queue_size=10)
    error_pub = rospy.Publisher('error_vector', Vector3Stamped, queue_size=10)

    # ROS services
    rospy.wait_for_service("/drone1/mavros1/cmd/arming")
    rospy.wait_for_service("/drone1/mavros1/set_mode")
    arming_client = rospy.ServiceProxy("/drone1/mavros1/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("/drone1/mavros1/set_mode", SetMode)

    # Gazebo teleport service
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    teleported = False  # flag to track if teleport happened

    # Camera intrinsics
    width = 800
    height = 600
    fov_x = 1.57 # horizontal_fov in radians

    f_x = (width / 2) / np.tan(fov_x / 2)
    f_y = f_x  # Assume square pixels

    # c_x = width / 2
    # c_y = height / 2


    # -----------------------------------------------
    # Controller configuration
    # -----------------------------------------------

    # General parameters
    l = 1           # distance potential exponent
    vLIM = 0.4        # velocity limit
    c1 = 0.5        # controller gain (was 0.55)

    # Altitute control
    h_star = 2      # desired altitute
    Kp = 1.2        # PID gains
    Kd = 0.1
    Ki = 0.1
    vLIMh = 1
    z_pid = cf.PID(Kp=Kp, Ki=Ki, Kd=Kd, dt=0.01, output_limits=(-vLIMh, vLIMh))
    yaw_pid = cf.PID(Kp=0.002, Ki=0.0, Kd=0.0001, dt=0.01, output_limits=(-0.15, 0.15))
    image_centre = 800/2

    # Start position drone
    t = np.array([0.0,3.0,0.0])

    # Incidence matrix
    B = np.array([
        [1, 0, -1],
        [-1, 1, 0],
        [0, -1, 1]
    ])
    B_bar = np.kron(B, np.eye(2))
    d = np.array([np.sqrt(9), np.sqrt(13), np.sqrt(13)])  # desired edge lengths

    # Reference fixed points (anchors)
    a = np.array([0.0,0.0])
    b = np.array([2.598,1.5])

    # Takeoff control
    takeoff_goal = [0.0, 0.0, h_star]
    pos_tolerance = 0.1

    # ROS rate & timing
    rate = rospy.Rate(20)
    prev_time = rospy.Time.now() if simulation else time.time()

    # # Time tracking
    # prev_time = rospy.Time.now() if simulation else time.time()

    # -----------------------------------------------
    # -----------------------------------------------

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
            # x = current_pose.pose.position.x + t[0]
            # y = current_pose.pose.position.y + t[1]
            h = current_pose.pose.position.z + t[2]

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
            z_pid.dt = dt
            yaw_pid.dt = dt

            # # Teleporting at t=150s (disturbance)
            # if not teleported and (current_time.to_sec()-t_start) >= 150.0:
            #     state_msg = ModelState()
            #     state_msg.model_name = 'iris1_custom'
            #     state_msg.pose.position.x = -5.0
            #     state_msg.pose.position.y = 8.0
            #     state_msg.pose.position.z = h
            #     state_msg.twist = Twist()
            #     state_msg.reference_frame = 'world'

            #     try:
            #         result = set_state(state_msg)
            #         rospy.loginfo("Teleported Iris at t=150s")
            #         teleported = True  # prevent it from repeating
            #     except rospy.ServiceException as e:
            #         rospy.logerr("Failed to teleport: %s" % e)

            # # Shut down at t=400s
            # if (current_time.to_sec()-t_start) >= 300.0:
            #     rospy.loginfo("[Drone1] Time limit reached. Shutting down simulation at t=300s")
            #     rospy.signal_shutdown("Simulation time limit reached")

            if new_data:
                new_data = False

                # Yaw control
                yaw_dot = yaw_pid.compute(image_centre - cx)

                # Control law parameters
                # p = np.array([x,y,a[0],a[1],b[0],b[1]])     # absolute positions
                # z = B_bar.T @ p                             # relative positions

                hoek = -theta
                ROT = np.array([
                    [np.cos(hoek), -np.sin(hoek)],
                    [np.sin(hoek),  np.cos(hoek)]
                ])
                # Define relative positions
                z1 = ROT @ (zd*np.array([-2*np.sin(theta), 0]))
                z2 = ROT @ (zd*np.array([np.sin(theta), -np.cos(theta)]))
                z3 = ROT @ (zd*np.array([np.sin(theta), np.cos(theta)]))
                z = np.concatenate((z1, z2, z3))
                # z = np.diag([ROT,ROT,ROT]) @ z

                z_norm = np.linalg.norm(z.reshape((3, 2)), axis=1) # link length
                e = z_norm**l - d**l                        # error     
                z_tilde = z_norm**(l-2)                     # norm-based weights
                Dz = cf.block_diag(z)                       # diagonal matrix
                Dz_tilde = np.diag(z_tilde)                 # idem

                p_dot = -c1 * B_bar @ Dz @ Dz_tilde @ e     # control law

                # Publish errors
                ep = z_norm - d
                msg = Vector3Stamped()
                msg.header.stamp = rospy.Time.now()
                msg.vector.x = ep[0]
                msg.vector.y = ep[1]
                msg.vector.z = ep[2]
                error_pub.publish(msg)

                # Record data
                log_data.append((current_time.to_sec()-t_start, ep[0], ep[1], ep[2], z_norm[0], z_norm[1], z_norm[2], 0.0, 0.0,h))
            
                # angle = np.arctan((image_centre - cx) / f_x)
                # v = np.array([p_dot[0], p_dot[1]])
                # R = np.array([
                #     [np.cos(angle), -np.sin(angle)],
                #     [np.sin(angle),  np.cos(angle)]
                # ])
                # v = R @ v
                # x_dot = np.clip(v[0], -vLIM, vLIM)
                # y_dot = np.clip(v[1], -vLIM, vLIM)

                x_dot = np.clip(p_dot[1], -vLIM, vLIM)
                y_dot = -np.clip(p_dot[0], -vLIM, vLIM)

                # if abs(image_centre - cx) < 15:
                #     x_dot = np.clip(p_dot[0], -vLIM, vLIM)
                #     y_dot = np.clip(p_dot[1], -vLIM, vLIM)
                #     # print(cx)
                # else:
                #     x_dot = 0.0
                #     y_dot = 0.0
            else:
                x_dot = 0.0
                y_dot = 0.0
                yaw_dot = 0.0

            # Control height
            z_dot = z_pid.compute(h_star - h)

            # Send data to drone
            twist = TwistStamped()
            twist.twist.linear.x = x_dot
            twist.twist.linear.y = y_dot
            twist.twist.linear.z = z_dot
            twist.twist.angular.z = yaw_dot
            rospy.set_param("/drone1/mavros1/setpoint_velocity/mav_frame", "BODY_NED")
            vel_pub.publish(twist)

           
        rate.sleep()

if __name__ == "__main__":
    main()
