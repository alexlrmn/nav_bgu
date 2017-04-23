#!/usr/bin/env python
from rospy import init_node, spin, Time, Publisher, Subscriber
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
import message_filters
from std_msgs.msg import String, Header
import yaml
from nav_bgu.msg import StringStamped
import numpy as np
import tf
from tf.transformations import quaternion_from_matrix, \
    translation_from_matrix, quaternion_from_matrix


import pprint

class qr_code_manager:

    def __init__(self):

        init_node("qr_code_manager", anonymous=True)

        self.markers = dict()
        self.init_markers()

        self.pub = Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=5)

        self.stamp_string_pub = Publisher("/visp_auto_tracker/code_message_stamped", StringStamped, queue_size=5)

        self.stamp_string_sub = Subscriber("/visp_auto_tracker/code_message", String, self.stamp_string_callback)
        self.stamp_position_sub = Subscriber("/visp_auto_tracker/object_position_covariance",
                                             PoseWithCovarianceStamped, self.stamp_position_callback)

        self.code_stamped = StringStamped()
        self.position_stamped = PoseWithCovarianceStamped()

        self.baseLinkFrameId = rospy.get_param('~base_link_frame_id', '/base_link')
        self.mapFrameId = rospy.get_param('~map_frame_id', '/map')

        self.tr = tf.TransformerROS()
        self.tl = tf.TransformListener()

    def stamp_string_callback(self, data):

        if data.data != '':
            h = Header()
            h.stamp = Time.now()

            self.code_stamped = StringStamped()
            self.code_stamped.header = h
            self.code_stamped.data = data.data

            self.execute()

    def stamp_position_callback(self, data):

        self.position_stamped = data
        self.execute()

    def execute(self):

        if self.position_stamped.header.stamp.secs == self.code_stamped.header.stamp.secs:
            self.tf_localization(self.code_stamped.data, self.position_stamped)

    def tf_localization(self, code, position):
        # Get position of observed marker

        pose = position.pose.pose
        object_matrix = np.matrix(self.tr.fromTranslationRotation
                                  ((pose.position.x, pose.position.y, pose.position.z),
                                   (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)))

        # Retrieve base link position
        try:
            t = self.tl.lookupTransform(self.baseLinkFrameId, 'kinect2_link',
                                   rospy.Time(0))
        except:
            rospy.logwarn("failed to retrieve camera position w.r.t. base_link")
            return

        blMc = np.matrix(self.tr.fromTranslationRotation(t[0], t[1]))

        # Create matrix from known position of the object
        wMo = np.matrix(np.identity(4))
        wMo_t = [0., 0., 0.]
        wMo_q = [0., 0., 0., 1.]
        wMo_t[0] = self.markers[code]['position']['x']
        wMo_t[1] = self.markers[code]['position']['y']
        wMo_t[2] = self.markers[code]['position']['z']
        wMo_q[0] = self.markers[code]['orientation']['x']
        wMo_q[1] = self.markers[code]['orientation']['y']
        wMo_q[2] = self.markers[code]['orientation']['z']
        wMo_q[3] = self.markers[code]['orientation']['w']
        wMo = np.matrix(self.tr.fromTranslationRotation(wMo_t, wMo_q))
        oMw = np.linalg.inv(wMo)

        # Compute the base link position w.r.t the world frame.
        blMw = blMc * object_matrix * oMw

        blMw_t = translation_from_matrix(blMw)
        blMw_q = quaternion_from_matrix(blMw)

        br = tf.TransformBroadcaster()

        br.sendTransform(blMw_t, blMw_q,
                         position.header.stamp,
                         self.baseLinkFrameId,
                         self.mapFrameId)
        print '1'
        pprint.pprint(blMw_t, indent=2)
        print '2'
        pprint.pprint(blMw_q, indent=2)



    def init_markers(self):

        with open("qr_codes.yaml") as f:
            self.markers = yaml.safe_load(f)


            # print self.markers['position_2']['orientation']['x']


def main():
    qr = qr_code_manager()

    try:
        spin()
    except KeyboardInterrupt:
        print 'Shutting down'


if __name__ == "__main__":
    main()
