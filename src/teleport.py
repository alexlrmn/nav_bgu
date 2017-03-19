#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Quaternion, Point

class teleport:

	def __init__(self):

		rospy.init_node("teleport", anonymous=True)

		self.tele_in = rospy.Subscriber("/teleport_cmd", String, self.call_back)
		self.robot_pose = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.curr_pose_callback)
		self.init_pose = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=5)

		self.pose_with_cov = PoseWithCovarianceStamped()

		self.loc = dict()
		self.loc["first_floor"] = Point(-2.71, -18.6, 0)
		self.loc["second_floor"] = Point(8.20, -22.6, 0)

		
	def call_back(self, msg):
		global pose_with_cov

		point = self.loc[str(msg.data)]

		#Get robot pose to init
		pose_with_cov.header.stamp = rospy.Time.now()
		pose_with_cov.pose.pose.position = point
		#Initiate robots position in new floor
		
		self.init_pose.publish(pose_with_cov)




	def curr_pose_callback(self, data):
		global pose_with_cov

		pose_with_cov = data


if __name__ == "__main__":

	teleport()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print 'Shutting down'