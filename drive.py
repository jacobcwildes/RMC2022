#!/usr/bin/env python
# Drive node.
# Receives commands from other nodes and operates motors, as well
# as calculating movement from cell to cell on the nav grid.

import rospy, sys
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from ursa.srv import DriveTo, DriveToResponse, Move, MoveResponse, Turn, MotorSpeed
from util import maestro

class Drive:

	def __init__(self):
		self.wheelDist = rospy.get_param('/wheel_dist', 0.48)
		self.wheelRadius = rospy.get_param('/wheel_radius', 0.10)
		self.maxSpeed = rospy.get_param('/max_speed', 1)
		self.motors = rospy.get_param('/motors')

		self.controller = None
		self.maestro_init()

		rospy.Subscriber('cmd_vel', Twist, self.cmd_vel)
		rospy.Service('drive_to', DriveTo, self.drive_to)
		rospy.Service('move', Move, self.move)
		rospy.Service('turn', Turn, self.turn)
		rospy.Service('start_digging', Trigger, self.start_digging)
		rospy.Service('stop_digging', Trigger, self.stop_digging)
		rospy.Service('dump_collector', Trigger, self.dump_collector)

		rospy.on_shutdown(self.on_exit)
		rospy.spin()


	def on_exit(self):
		"""Shut motors off and close controller."""
		if self.controller is not None:
			for motor in self.motors:
				self.set_motor(motor, motor['center'])
			self.controller.close()
	

	def cmd_vel(self, data):
		"""Take a Twist message and drive the motors."""
		linearVel = data.linear.x / self.maxSpeed
		angularVel = data.angular.z / self.maxSpeed
		motorL, motorR = self.motor_mix(linearVel, angularVel)
		motorL = self.maestro_scale(self.motors['LEFT'], motorL)
		motorR = self.maestro_scale(self.motors['RIGHT'], motorR)
		self.set_motor(self.motors['LEFT'], motorL)
		self.set_motor(self.motors['RIGHT'], motorR)
		rospy.loginfo("Left: %f Right: %f", motorL, motorR)
	

	def motor_mix(self, linearVel, angularVel):
		"""Takes a velocity percentage (-1.0 to 1.0) for
		linear and angular, mixing them into a motor
		power ratio."""
		r = 1 * linearVel
		l = angularVel
		v = (1 - abs(r)) * l + l
		w = (1 - abs(l)) * r + r
		motorR = -(v + w) / 2
		motorL = -(v - w) / 2
		return (motorL, motorR)


	def maestro_scale(self, motor, value):
		"""Convert -1.0 to 1.0 into Maestro target values."""
		if value >= 0:
			r = int(motor['center'] + (motor['max'] - motor['center']) * value)
		else:
			r = int(motor['center'] + (motor['center'] - motor['min']) * value)
		return r


	def set_motor(self, motor, value):
		"""Set the Maestro target for a motor"""
		if self.controller is not None:
			self.controller.setTarget(motor['channel'], value)


	def maestro_init(self):
		"""Initialize the Maestro controller and set some
		starting values on the motor channels."""
		try:
			c = maestro.Controller()
			self.controller = c
			for name, motor in self.motors.items():
				c.setAccel(motor['channel'], motor['accel'])
				c.setSpeed(motor['channel'], motor['speed'])
		except:
			rospy.logerr("Could not connect to Maestro.")


	def drive_to(self, data):
		"""Takes a Pose object (data.target) and drives the robot to it.
		Returns a Pose estimating the new position by odometry."""
		return DriveToResponse(data.target)


	def move(self, data):
		"""Drive forward or background n meters (data.distance).
		Negative for reverse.
		Returns meters traveled, by motor odometry."""
		return


	def turn(self, data):
		"""Turn n degrees (data.degrees). Negative to reverse direction."""
		return


	def start_digging(self):
		return


	def stop_digging(self):
		return


	def dump_collector(self):
		return



if __name__ == '__main__':
	try:
		rospy.init_node('drive')
		Drive()
	except rospy.ROSInterruptException:
		pass