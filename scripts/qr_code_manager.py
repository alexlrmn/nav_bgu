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
import rospkg


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



        self.d = {}

    def stamp_string_callback(self, data):

        if data.data != '':

            code = data.data

            if (code == 'http://www.irisa.fr/lagadic/visp'):
                code = code.split(":")[1][2:]


            h = Header()
            h.stamp = Time.now()

            self.code_stamped = StringStamped()
            self.code_stamped.header = h
            self.code_stamped.data = code

            self.execute()

    def stamp_position_callback(self, data):

        self.position_stamped = data
        self.execute()

    def execute(self):

        if self.position_stamped.header.stamp.secs == self.code_stamped.header.stamp.secs:
            self.tf_localization(self.code_stamped.data, self.position_stamped)

    def tf_localization(self, code, position):
        # Get position of observed marker

        if code not in self.d:

            p = position.pose.pose
            self.d[code] = {}
            self.d[code]['position'] = {}
            self.d[code]['orientation'] = {}
            """self.d[code]['position']['x'] = {}
            self.d[code]['position']['y'] = {}
            self.d[code]['position']['z'] = {}
            self.d[code]['orientation']['x'] = {}
            self.d[code]['orientation']['y'] = {}
            self.d[code]['orientation']['z'] = {}
            self.d[code]['orientation']['w'] = {}"""

            self.d[code]['position']['x'] = p.position.x
            self.d[code]['position']['y'] = p.position.y
            self.d[code]['position']['z'] = p.position.z
            self.d[code]['orientation']['x'] = p.orientation.x
            self.d[code]['orientation']['y'] = p.orientation.y
            self.d[code]['orientation']['z'] = p.orientation.z
            self.d[code]['orientation']['w'] = p.orientation.w

        pose = position.pose.pose
        object_matrix = np.matrix(self.tr.fromTranslationRotation
                                  ((pose.position.x, pose.position.y, pose.position.z),
                                   (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)))

        # Retrieve base link position
        """
        try:
            self.tl.waitForTransform(self.baseLinkFrameId, 'kinect2_link', rospy.Time(0))
            t = self.tl.lookupTransform(self.baseLinkFrameId, 'kinect2_link',
                                             rospy.Time(0))
        except:
            rospy.logwarn("failed to retrieve camera position w.r.t. base_link")
            return
        """

        #blMc = np.matrix(self.tr.fromTranslationRotation(t[0], t[1]))
        blMc = np.matrix(self.tr.fromTranslationRotation((0, 0, 1), (0, 0, 0, 1)))

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

        #print blMw_t
        #print blMw_q

        """
        br = tf.TransformBroadcaster()

        br.sendTransform(blMw_t, blMw_q,
                         position.header.stamp,
                         self.baseLinkFrameId,
                         self.mapFrameId)
        """

        pos = PoseWithCovarianceStamped()
        pos.header.stamp = rospy.Time(0)
        pos.pose.covariance = position.pose.covariance

        pos.pose.pose.position.x = blMw_t[0]
        pos.pose.pose.position.y = blMw_t[1]
        pos.pose.pose.position.z = blMw_t[2]

        pos.pose.pose.orientation.x = blMw_q[0]
        pos.pose.pose.orientation.y = blMw_q[1]
        pos.pose.pose.orientation.z = blMw_q[2]
        pos.pose.pose.orientation.w = blMw_q[3]

        self.pub.publish(pos)

        rospy.sleep(3)
        """
        print '1'
        pprint.pprint(blMw_t, indent=2)
        print '2'
        pprint.pprint(blMw_q, indent=2)
        """


    def init_markers(self):

        rospack = rospkg.RosPack()

        with open(rospack.get_path("nav_bgu") + "/scripts/qr_codes.yaml") as f:
            self.markers = yaml.safe_load(f)


            # print self.markers['position_2']['orientation']['x']

    def exitf(self):
        rospack = rospkg.RosPack()
        with open(rospack.get_path("nav_bgu") + "/scripts/read_qr.yaml", 'wr') as f:
            yaml.safe_dump(self.d, f, default_flow_style=False)

def main():
    qr = qr_code_manager()

    try:
        spin()
    except KeyboardInterrupt:

        print 'Shutting down'

    print 'bye'
    qr.exitf()


if __name__ == "__main__":
    main()
