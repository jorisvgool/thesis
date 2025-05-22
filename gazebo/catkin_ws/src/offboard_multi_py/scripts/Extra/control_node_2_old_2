#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

def main():
    rospy.init_node("control_node_2", anonymous=True)

    rospy.Subscriber("/drone2/mavros2/state", State, state_cb)
    local_pos_pub = rospy.Publisher("/drone2/mavros2/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/drone2/mavros2/cmd/arming")
    rospy.wait_for_service("/drone2/mavros2/set_mode")

    arming_client = rospy.ServiceProxy("/drone2/mavros2/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("/drone2/mavros2/set_mode", SetMode)

    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    rate = rospy.Rate(20)

    while not rospy.is_shutdown() and not current_state.connected:
        rospy.loginfo_throttle(2, "[Drone2] Waiting for FCU connection...")
        rate.sleep()

    rospy.loginfo("[Drone2] FCU connected")

    for _ in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()

    last_req = rospy.Time.now()

    while not rospy.is_shutdown():
        local_pos_pub.publish(pose)

        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req > rospy.Duration(5.0)):
            if set_mode_client(custom_mode="OFFBOARD").mode_sent:
                rospy.loginfo("[Drone2] OFFBOARD mode set")
            last_req = rospy.Time.now()

        elif not current_state.armed and (rospy.Time.now() - last_req > rospy.Duration(5.0)):
            if arming_client(True).success:
                rospy.loginfo("[Drone2] Drone armed")
            last_req = rospy.Time.now()

        rate.sleep()

if __name__ == "__main__":
    main()
