#!/usr/bin/python2.7
# -*- coding: utf-8 -*-
import rospy
from rover_ctrl.msg import Rov_ctrl
from std_msgs.msg import String
import os


angle_amplitude = 40 #°
angle_duration = 10 #s
torque_amplitude = 5 #N.m
torque_duration = 1 #s


def callback_info( msg ):
	print( msg.data )


try :

	pub = rospy.Publisher( 'nav_ctrl', Rov_ctrl, queue_size=10 )
	rospy.init_node( 'crawling_gait', disable_signals=True )
	sub_info = rospy.Subscriber( 'nav_info_string', String, callback_info )

	repeat_period = rospy.get_param( '~repeat_period', 500 )*1e-3

	cmd = Rov_ctrl()

	cmd.engaged = True
	cmd.crawling_mode = True
	cmd.rate_mode = False
	cmd.speed = 0.0
	cmd.torque = 0.0


	def ask_torque( torque, duration ) :

		n_full_periods = int( duration//( repeat_period ) )
		extra_time = duration%( repeat_period )

		cmd.torque = torque

		for _ in range( n_full_periods ) :

			cmd.header.stamp = rospy.Time.now()
			pub.publish( cmd )

			rospy.sleep( repeat_period )

		if extra_time > 0 :

			cmd.header.stamp = rospy.Time.now()
			pub.publish( cmd )

			rospy.sleep( extra_time )


	def ask_angle( angle, duration ) :

		n_full_periods = int( duration//( repeat_period ) )
		extra_time = duration%( repeat_period )

		cmd.steer = angle

		for _ in range( n_full_periods ) :

			cmd.header.stamp = rospy.Time.now()
			pub.publish( cmd )

			rospy.sleep( repeat_period )

		if extra_time > 0 :

			cmd.header.stamp = rospy.Time.now()
			pub.publish( cmd )

			rospy.sleep( extra_time )


	#ask_torque( -torque_amplitude, torque_duration )
	ask_angle( angle_amplitude, angle_duration/2 )

	while not rospy.is_shutdown() :

		#ask_torque( torque_amplitude, torque_duration )
		ask_angle( -angle_amplitude, angle_duration )

		#ask_torque( -torque_amplitude, torque_duration )
		ask_angle( angle_amplitude, angle_duration )


except KeyboardInterrupt :
	cmd.engaged = False

	cmd.header.stamp = rospy.Time.now()
	pub.publish( cmd )

	rospy.signal_shutdown( 'Received SIGINT' )
