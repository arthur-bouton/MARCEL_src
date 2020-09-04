#!/usr/bin/python2.7
# -*- coding: utf-8 -*-
import rospy
from rover_ctrl.msg import Rov_ctrl, Joints_info
from std_msgs.msg import String
import os
import numpy as np


angle_amplitude = 35 #°
angle_margin = 0.5 #°
angle_rate = 10 #°/s
angle_rate_up_ramp_duration = 1 #s
angle_rate_down_ramp_duration = 1 #s

torque_low = 10 #N.m
torque_high = 15 #N.m
torque_margin = 1 #N.m


actual_angle = 0
actual_torque = 0

def callback_joints( msg ):
	global actual_angle, actual_torque

	actual_angle = msg.cj_angle
	actual_torque = msg.sea_torque


def callback_string( msg ):
	print( msg.data )


try :

	cmd = Rov_ctrl()

	cmd.engaged = True
	cmd.crawling_mode = True
	cmd.rate_mode = True
	cmd.speed = 0.0
	cmd.torque = 0.0

	pub = rospy.Publisher( 'nav_ctrl', Rov_ctrl, queue_size=10 )
	rospy.init_node( 'crawling_gait', disable_signals=True )
	joints_info = rospy.Subscriber( 'joints_info', Joints_info, callback_joints )
	string_info = rospy.Subscriber( 'nav_info_string', String, callback_string )

	period = rospy.get_param( '~period', 100 )*1e-3


	def publish_cmd() :
		cmd.header.stamp = rospy.Time.now()
		pub.publish( cmd )
		print rospy.get_time(), cmd.steer, cmd.torque, actual_angle, actual_torque


	def get_desired_torque( sign ) :

		desired_torque = torque_high - ( torque_high - torque_low )*abs( actual_angle )/angle_amplitude

		return sign*desired_torque


	def target_torque( torque ) :

		cmd.torque = torque

		while ( -1 if torque < 0 else 1 )*actual_torque + torque_margin < abs( torque ) :

			publish_cmd()

			rospy.sleep( period )


	def target_angle( angle ) :

		for rate in np.linspace( 0, ( -1 if angle < 0 else 1 )*angle_rate, int( angle_rate_up_ramp_duration//period ) ) :

			if ( -1 if angle < 0 else 1 )*actual_angle + angle_margin >= abs( angle ) :
				break

			cmd.steer = rate
			cmd.torque = get_desired_torque( -1 if angle > 0 else 1 )

			publish_cmd()

			rospy.sleep( period )

		while ( -1 if angle < 0 else 1 )*actual_angle + angle_margin < abs( angle ) :

			cmd.torque = get_desired_torque( -1 if angle > 0 else 1 )

			publish_cmd()

			rospy.sleep( period )

		for rate in np.linspace( ( -1 if angle < 0 else 1 )*angle_rate, 0, int( angle_rate_down_ramp_duration//period ) ) :

			cmd.steer = rate
			cmd.torque = get_desired_torque( -1 if angle > 0 else 1 )

			publish_cmd()

			rospy.sleep( period )


	target_torque( -torque_low )
	target_angle( angle_amplitude )

	while not rospy.is_shutdown() :

		target_torque( torque_low )
		target_angle( -angle_amplitude )

		target_torque( -torque_low )
		target_angle( angle_amplitude )


except KeyboardInterrupt :
	cmd.engaged = False

	publish_cmd()

	rospy.signal_shutdown( 'Received SIGINT' )
