#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$


import rospy
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import *


class object:
	def __init__(self):
		self.pos1 = PoseStamped()
		self.pos2 = PoseStamped()
		self.pos3 = PoseStamped()
		self.status = 0

	def get_status(self, msg):
		self.status = msg.status_list[0].status if len(msg.status_list) else 0
		#the above (if) exists in case the array is zero, not to 			publish indexing errors
		#print(self.status)

	def get_new_goal(self, msg):
		self.pos1 = msg
		#print(self.pos1)

	def calculate(self):
		self.pos2 = self.pos1
		self.pos3 = self.pos1
		orientation_list = [self.pos1.pose.orientation.x, self.pos1.pose.orientation.y, self.pos1.pose.orientation.z, self.pos1.pose.orientation.w]
		(roll, pitch, yaw) = euler_from_quaternion (orientation_list)

		self.pos2.pose.position.x = self.pos1.pose.position.x + hypot(cos(yaw+3.14159265359),sin(yaw+3.14159265359))
		self.pos2.pose.position.y = self.pos1.pose.position.y + hypot(cos(yaw+1.57079632679),sin(yaw+1.57079632679))
		self.pos3.pose.position.x = self.pos1.pose.position.x + (cos(yaw-1.57079632679)/fabs(cos(yaw-1.57079632679)) * sin(yaw-1.57079632679)/fabs(sin(yaw-1.57079632679)) *       hypot(cos(yaw-1.57079632679),sin(yaw-1.57079632679)))
		self.pos3.pose.position.y = self.pos1.pose.position.y + (cos(yaw+3.14159265359)/fabs(cos(yaw+3.14159265359)) * sin(yaw+3.14159265359)/fabs(sin(yaw+3.14159265359)) *       hypot(cos(yaw+3.14159265359),sin(yaw+3.14159265359)))
		#print(self.pos2.pose.position.x)
		#print(self.pos2.pose.position.y)
		#print(self.pos3.pose.position.x)
		#print(self.pos3.pose.position.y)

def formation_controller():
	rospy.init_node('formation_controller')
	drone = object()
	rospy.Subscriber('quadrotor1/move_base/status', GoalStatusArray, drone.get_status)
	rospy.Subscriber('quadrotor1/move_base_simple/goal', PoseStamped, drone.get_new_goal)
	pub1 = rospy.Publisher('quadrotor2/move_base_simple/goal', PoseStamped, queue_size = 20)
	pub2 = rospy.Publisher('quadrotor3/move_base_simple/goal', PoseStamped, queue_size = 20)
	rate = rospy.Rate(5) # 5hz
	while not rospy.is_shutdown():
		if drone.pos1.header.seq == 1:
			drone.calculate()
			#print("esteila")
			print(drone.pos3)
			print(drone.pos1)
			pub1.publish(drone.pos2)
			pub2.publish(drone.pos3)
			#time.sleep(0.1)
		rate.sleep()

if __name__ == '__main__':
    try:
        formation_controller()
    except rospy.ROSInterruptException:
        pass
