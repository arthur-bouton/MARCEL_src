#!/usr/bin/python2.7
# -*- coding: utf-8 -*-
import rospy
from rover_ctrl.msg import Rov_ctrl, Joints_info
from std_msgs.msg import String
import os


angle_amplitude = 40 #°
torque_amplitude = 5 #N.m
angle_margin = 0.5 #°
torque_margin = 0.5 #N.m


actual_angle = 0
actual_torque = 0

def callback_joints( msg ):
	global actual_angle, actual_torque

	actual_angle = msg.cj_angle
	actual_torque = msg.sea_torque


def callback_string( msg ):
	print( msg.data )


try :

	pub = rospy.Publisher( 'nav_ctrl', Rov_ctrl, queue_size=10 )
	rospy.init_node( 'crawling_gait', disable_signals=True )
	joints_info = rospy.Subscriber( 'joints_info', Joints_info, callback_joints )
	string_info = rospy.Subscriber( 'nav_info_string', String, callback_string )

	period = rospy.get_param( '~period', 100 )*1e-3

	cmd = Rov_ctrl()

	cmd.engaged = True
	cmd.crawling_mode = True
	cmd.rate_mode = False
	cmd.speed = 0.0
	cmd.torque = 0.0


	def target_torque( torque ) :

		cmd.torque = torque

		while ( -1 if torque < 0 else 1 )*actual_torque + torque_margin < abs( torque ) :

			cmd.header.stamp = rospy.Time.now()
			pub.publish( cmd )

			rospy.sleep( period )


	def target_angle( angle ) :

		cmd.steer = angle

		while ( -1 if angle < 0 else 1 )*actual_angle + angle_margin < abs( angle ) :

			cmd.header.stamp = rospy.Time.now()
			pub.publish( cmd )

			rospy.sleep( period )


	#target_torque( -torque_amplitude )
	target_angle( angle_amplitude )

	while not rospy.is_shutdown() :

		#target_torque( torque_amplitude )
		target_angle( -angle_amplitude )

		#target_torque( -torque_amplitude )
		target_angle( angle_amplitude )


except KeyboardInterrupt :
	cmd.engaged = False

	cmd.header.stamp = rospy.Time.now()
	pub.publish( cmd )

	rospy.signal_shutdown( 'Received SIGINT' )
