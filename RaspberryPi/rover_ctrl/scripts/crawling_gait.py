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

torque_start  = 5 #N.m
torque_middle = 20 #N.m
torque_end    = 15 #N.m
torque_offset = 5 #°
torque_margin = 1 #N.m



cmd = Rov_ctrl()
cmd.engaged = True
cmd.crawling_mode = True
cmd.rate_mode = True
cmd.speed = 0.0
cmd.torque = 0.0


actual_angle = 0.0
actual_torque = 0.0

def callback_joints( msg ):
	global actual_angle, actual_torque

	actual_angle = msg.cj_angle
	actual_torque = msg.sea_torque


def callback_string( msg ):
	print( msg.data )


pub = rospy.Publisher( 'nav_ctrl', Rov_ctrl, queue_size=10 )
rospy.init_node( 'crawling_gait', disable_signals=True )
joints_info = rospy.Subscriber( 'joints_info', Joints_info, callback_joints )
string_info = rospy.Subscriber( 'nav_info_string', String, callback_string )

freq = rospy.get_param( '~freq', 10 )
period = rospy.Rate( freq )


def publish_cmd() :
	cmd.header.stamp = rospy.Time.now()
	pub.publish( cmd )
	print rospy.get_time(), cmd.steer, cmd.torque, actual_angle, actual_torque


def get_desired_torque( rate_sign ) :

	if rate_sign*actual_angle < -torque_offset :
		torque_edge = torque_start
		offset_sign = 1
	else :
		torque_edge = torque_end
		offset_sign = -1

	desired_torque = torque_middle - ( torque_middle - torque_edge )*abs( actual_angle + rate_sign*torque_offset )/( angle_amplitude - offset_sign*torque_offset )

	return -rate_sign*desired_torque


def steer( target_angle ) :

	if ( -1 if target_angle < 0 else 1 )*actual_angle + angle_margin >= abs( target_angle ) :
		return

	rate_sign = 1 if target_angle > actual_angle else -1


	# Get close to the desired torque before starting to move:
	cmd.torque = get_desired_torque( rate_sign )

	while ( -1 if cmd.torque < 0 else 1 )*actual_torque + torque_margin < abs( cmd.torque ) :

		publish_cmd()
		period.sleep()


	# Gradually increase the steering rate:
	for rate in np.linspace( 0, rate_sign*angle_rate, max( 2, int( angle_rate_up_ramp_duration*freq ) ) )[1:] :

		if ( -1 if target_angle < 0 else 1 )*actual_angle + angle_margin >= abs( target_angle ) :
			break

		cmd.steer = rate
		cmd.torque = get_desired_torque( rate_sign )

		publish_cmd()
		period.sleep()


	# Move at the maximum steering rate until the target angle is reached:
	while ( -1 if target_angle < 0 else 1 )*actual_angle + angle_margin < abs( target_angle ) :

		cmd.torque = get_desired_torque( rate_sign )

		publish_cmd()
		period.sleep()


	# Gradually decrease the steering rate:
	for rate in np.linspace( rate_sign*angle_rate, 0, max( 2, int( angle_rate_down_ramp_duration*freq ) ) )[1:] :

		cmd.steer = rate
		cmd.torque = get_desired_torque( rate_sign )

		publish_cmd()
		period.sleep()


try :

	steer( angle_amplitude )

	while not rospy.is_shutdown() :

		steer( -angle_amplitude )

		steer( angle_amplitude )


except KeyboardInterrupt :
	cmd.engaged = False

	publish_cmd()

	rospy.signal_shutdown( 'Received SIGINT' )
