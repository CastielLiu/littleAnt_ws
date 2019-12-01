#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from little_ant_msgs.msg import Lane
import math

def talker():
	rospy.init_node('talker', anonymous=True)

	pub = rospy.Publisher('/lane', Lane, queue_size=10)
	rate = rospy.Rate(20) # 10hz
	
	lane_msg = Lane()
	
	distance_from_center = 0.0
	
	sign = 1
	
	while not rospy.is_shutdown():
		lane_msg.header.stamp = rospy.Time.now()
		
		lane_msg.distance_from_center = distance_from_center
		lane_msg.included_angle = 0.0*math.pi/180.0
		
		"""
		distance_from_center = distance_from_center + 0.001 * sign
		
		if(distance_from_center>0.5):
			sign = -1
		elif(distance_from_center < -0.5):
			sign = 1
		
		#distance_from_center = -0.3
		"""
		pub.publish(lane_msg)
		
		rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
