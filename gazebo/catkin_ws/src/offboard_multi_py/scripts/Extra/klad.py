#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import tf
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np

class StereoImageCombiner:
    def __init__(self):
        rospy.init_node('stereo_image_combiner', anonymous=True)

        # Topics for left and right cameras
        self.left_image_sub = Subscriber('/iris1_custom/iris_camera/camera/image_raw', Image)
        self.right_image_sub = Subscriber('/iris2_custom/iris_camera/camera/image_raw', Image)

        # TF listener to get relative transform
        self.tf_listener = tf.TransformListener()

        # CV bridge
        self.bridge = CvBridge()

        # Stereo publisher
        self.stereo_pub = rospy.Publisher('/stereo_image', Image, queue_size=1)

        # Synchronize image topics
        self.sync = ApproximateTimeSynchronizer(
            [self.left_image_sub, self.right_image_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.image_callback)

    def image_callback(self, left_msg, right_msg):
        try:
            # Convert images to OpenCV format
            left_img = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding='bgr8')
            right_img = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='bgr8')

            # Get transform from drone2 to drone1 (used to adjust stereo rectification)
            (trans, rot) = self.tf_listener.lookupTransform(
                '/iris1_custom', '/iris2_custom', rospy.Time(0))

            # Optional: use trans and rot to calibrate or align, if needed

            # Combine the images side by side into a stereo image
            stereo_image = np.hstack((left_img, right_img))

            # Convert back to ROS image and publish
            stereo_msg = self.bridge.cv2_to_imgmsg(stereo_image, encoding='bgr8')
            self.stereo_pub.publish(stereo_msg)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException) as e:
            rospy.logwarn(f"TF Exception: {e}")
        except Exception as e:
            rospy.logerr(f"Error processing stereo images: {e}")

if __name__ == '__main__':
    try:
        combiner = StereoImageCombiner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
