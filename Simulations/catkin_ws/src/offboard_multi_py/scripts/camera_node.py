#!/usr/bin/env python

import rospy
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped

current_state = None

def state_cb(msg):
    global current_state
    current_state = msg

def main():
    rospy.init_node('camera_node', anonymous=True)
    drone_id = rospy.get_param("~drone_id", "drone1")

    state_sub = rospy.Subscriber(f"/{drone_id}/mavros/state", State, state_cb)
    local_pos_pub = rospy.Publisher(f"/{drone_id}/mavros/setpoint_position/local", PoseStamped, queue_size=10)

    arming_client = rospy.ServiceProxy(f"/{drone_id}/mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy(f"/{drone_id}/mavros/set_mode", SetMode)

    rate = rospy.Rate(20)

    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2.5  # Takeoff altitude

    # Wait for FCU connection
    while not rospy.is_shutdown() and (current_state is None or not current_state.connected):
        rate.sleep()

    # Send a few setpoints before starting
    for _ in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()

    # Set mode to OFFBOARD
    set_mode_client(base_mode=0, custom_mode="OFFBOARD")
    # Arm the drone
    arming_client(True)

    rospy.loginfo(f"[{drone_id}] Armed and OFFBOARD mode set!")

    while not rospy.is_shutdown():
        local_pos_pub.publish(pose)
        rate.sleep()

if __name__ == "__main__":
    main()
