#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan

class manipulate:

	def __init__(self):
		rospy.init_node("laser_manipulation", anonymous=True)

		self.laser_scan_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
		self.depth_to_laser_callback = rospy.Subscriber("/camera_scan", LaserScan, self.dtl_callback)
		self.new_laser_pub = rospy.Publisher("/new_scan", LaserScan, queue_size=5)

		self.laser_msg = None
		self.dlt_msg = None


	def laser_callback(self, msg):
		self.laser_msg = msg
		if self.dlt_msg != None:
			new_laser_ray = [i for i in range(len(self.laser_msg.ranges))]
			#print len(new_laser_ray)
			#print len(self.laser_msg.ranges)
			#print len(self.dlt_msg.ranges)
			for i in range(len(self.laser_msg.ranges)):
				new_laser_ray[i] = min(self.laser_msg.ranges[i], self.dlt_msg.ranges[i])
			
			self.laser_msg.ranges = new_laser_ray
			self.new_laser_pub.publish(self.laser_msg)

		#Print msg content
		#print 'laser ' + str(len(self.laser_msg.ranges))


	def dtl_callback(self, msg):
		self.dlt_msg = msg
		if None != self.laser_msg:
			length_side = (len(self.dlt_msg.ranges) - len(self.laser_msg.ranges)) / 2
			self.dlt_msg.ranges = self.dlt_msg.ranges[length_side:length_side + len(self.laser_msg.ranges)]
			#print len(self.dlt_msg.ranges)
		#Print msg content
		#print 'camera ' + str(len(laser_msg.ranges))

if __name__ == "__main__":

	manipulate()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print 'Shutting down'