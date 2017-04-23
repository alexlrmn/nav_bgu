#!/usr/bin/env python


import rospy
import numpy as np
import tf
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_matrix, \
    translation_from_matrix, quaternion_from_matrix

class marker_localization:

    rospy.init_node('marker_localization', anonymous=True)






def main():
    ml = marker_localization()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print 'Marker localization is shutting down'


if __name__ == '__main__':
    main()
