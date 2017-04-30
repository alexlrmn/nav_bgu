#!/usr/bin/env python
import rospy
import tf
rospy.init_node('test', anonymous=True)
tl = tf.TransformListener()
baseLinkFrameId = 'base_link'
t = None

while not rospy.is_shutdown():
    try:
        #tl.waitForTransform(baseLinkFrameId, 'kinect2_link', rospy.Time(0))
        t = tl.lookupTransform(baseLinkFrameId, 'kinect2_link',
                               rospy.Time(0))
    except:
        print("failed to retrieve camera position w.r.t. base_link")

    if t is not None:
        print t[0]
try:
    rospy.spin()
except KeyboardInterrupt:
    print 'bye'


