#!/usr/bin/env python

import rospy
from mavros_msgs.srv import ParamSet, ParamSetRequest
from mavros_msgs.msg import State

class BaroOffsetResetter:
    def __init__(self):
        rospy.init_node('reset_baro_offset')

        self.connected = False
        self.ns = rospy.get_namespace()

        rospy.wait_for_service(self.ns + 'mavros/param/set')
        self.param_set = rospy.ServiceProxy(self.ns + 'mavros/param/set', ParamSet)

        rospy.Subscriber(self.ns + 'mavros/state', State, self.state_cb)

        rospy.loginfo("Waiting for FCU connection...")
        rospy.spin()

    def state_cb(self, msg):
        if msg.connected and not self.connected:
            rospy.loginfo("Connected! Setting CAL_BARO1_OFF to 0.0...")
            self.connected = True

            req = ParamSetRequest()
            req.param_id = "CAL_BARO1_OFF"
            req.value.real = 0.0
            self.param_set.call(req)

            rospy.loginfo("Barometer offset reset successful!")
            rospy.signal_shutdown("Done.")

if __name__ == "__main__":
    BaroOffsetResetter()
