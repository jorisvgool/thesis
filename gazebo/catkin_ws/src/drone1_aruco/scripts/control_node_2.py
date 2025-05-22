#!/usr/bin/env python3

import rospy
import numpy as np
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import TwistStamped, PoseStamped, Vector3Stamped
import time

current_state = State()
current_pose1 = None  # This drone
current_pose2 = None  # Other drone

# States
TAKEOFF = 0
FORMATION = 1
control_state = TAKEOFF

def state_cb(msg):
    global current_state
    current_state = msg

class PID:
    def __init__(self, Kp, Ki, Kd, dt, output_limits=(None, None)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0
        self.output_limits = output_limits

    def compute(self, error):
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error

        lower, upper = self.output_limits
        if lower is not None:
            output = max(lower, output)
        if upper is not None:
            output = min(upper, output)

        return output

def pose_cb1(msg):
    global current_pose1
    current_pose1 = msg

def pose_cb2(msg):
    global current_pose2
    current_pose2 = msg

def distance_to_goal(goal):
    dx = current_pose2.pose.position.x - goal[0]
    dy = current_pose2.pose.position.y - goal[1]
    dz = current_pose2.pose.position.z - goal[2]
    return np.sqrt(dx*dx + dy*dy + dz*dz)

def main():
    global control_state
    rospy.init_node("control_node_2", anonymous=True)

    # Subscribers
    rospy.Subscriber("/drone2/mavros2/state", State, state_cb)
    rospy.Subscriber("/drone1/mavros1/local_position/pose", PoseStamped, pose_cb1)
    rospy.Subscriber("/drone2/mavros2/local_position/pose", PoseStamped, pose_cb2)

    # Publishers
    vel_pub = rospy.Publisher("/drone2/mavros2/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    pos_pub = rospy.Publisher("/drone2/mavros2/setpoint_position/local", PoseStamped, queue_size=10)
    # error_pub = rospy.Publisher('error_vector', Vector3Stamped, queue_size=10)

    # Services
    rospy.wait_for_service("/drone2/mavros2/cmd/arming")
    rospy.wait_for_service("/drone2/mavros2/set_mode")
    arming_client = rospy.ServiceProxy("/drone2/mavros2/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("/drone2/mavros2/set_mode", SetMode)

    # Controller parameters
    l = 1
    c1 = 0.03
    h_star = 2  # desired height

    # Incidence matrix
    B = np.array([
        [1, 0, -1],
        [-1, 1, 0],
        [0, -1, 1]
    ])
    I2 = np.eye(2)
    B_bar = np.kron(B, I2)
    d = np.array([2, np.sqrt(5), np.sqrt(5)])  # desired edge lengths

    takeoff_goal = [0.0, 0.0, h_star]  # target point for takeoff
    pos_tolerance = 0.1  # meters
    rate = rospy.Rate(50)  # 20Hz

    # Time tracking
    prev_time = time.time()
    z_pid = PID(Kp=1.2, Ki=0.1, Kd=0.1, dt=0.02, output_limits=(-1.0, 1.0))


    while not rospy.is_shutdown() and not current_state.connected:
        rospy.loginfo_throttle(2, "[Drone2] Waiting for FCU connection...")
        rate.sleep()

    rospy.loginfo("[Drone2] FCU connected")

    # Send some dummy commands first
    pose_cmd = PoseStamped()
    pose_cmd.pose.position.x = takeoff_goal[0]
    pose_cmd.pose.position.y = takeoff_goal[1]
    pose_cmd.pose.position.z = takeoff_goal[2]

    for _ in range(100):
        pos_pub.publish(pose_cmd)
        rate.sleep()

    last_req = rospy.Time.now()

    while not rospy.is_shutdown():
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req > rospy.Duration(5.0)):
            if set_mode_client(custom_mode="OFFBOARD").mode_sent:
                rospy.loginfo("[Drone2] OFFBOARD mode set")
            last_req = rospy.Time.now()

        elif not current_state.armed and (rospy.Time.now() - last_req > rospy.Duration(5.0)):
            if arming_client(True).success:
                rospy.loginfo("[Drone2] Drone armed")
            last_req = rospy.Time.now()
            
        if control_state == TAKEOFF:
            pos_pub.publish(pose_cmd)
            distance = distance_to_goal(takeoff_goal)

            if distance < pos_tolerance:
                rospy.loginfo("[Drone2] Takeoff completed, switching to formation control...")
                control_state = FORMATION

        elif control_state == FORMATION:
            # Formation control logic
            h = current_pose2.pose.position.z
            x1 = current_pose1.pose.position.x
            y1 = current_pose1.pose.position.y
            x2 = current_pose2.pose.position.x
            y2 = current_pose2.pose.position.y

            # Calculate dt
            current_time = time.time()
            dt = current_time - prev_time
            z_pid.dt = dt
            prev_time = current_time
            
            # Control law parameters
            p = np.array([0,2,x2,y2,2,1])
            z = B_bar.T @ p
            z_matrix = z.reshape((3, 2))
            e = np.linalg.norm(z_matrix, axis=1)**l - d**l
            z_tilde = np.linalg.norm(z_matrix, axis=1)**(l-2)
            Dz = np.zeros((6, 3))
            Dz[0:2, 0] = z[0:2]
            Dz[2:4, 1] = z[2:4]
            Dz[4:6, 2] = z[4:6]
            Dz_tilde = np.diag(z_tilde)

            p_dot = -c1 * B_bar @ Dz @ Dz_tilde @ e
            x_dot = p_dot[2]
            y_dot = p_dot[3]

            # Publish errors
            # msg = Vector3Stamped()
            # msg.header.stamp = rospy.Time.now()
            # msg.vector.x = e[0]
            # msg.vector.y = e[1]
            # msg.vector.z = e[2]
            # error_pub.publish(msg)

            # Control height
            z_dot = z_pid.compute(h_star - h)

            # Send data to drone
            twist = TwistStamped()
            twist.twist.linear.x = x_dot
            twist.twist.linear.y = y_dot
            twist.twist.linear.z = z_dot

            # rospy.loginfo(f"[Drone2] x_dot: {x_dot:.3f}, y_dot: {y_dot:.3f}")

            vel_pub.publish(twist)

        rate.sleep()

if __name__ == "__main__":
    main()
