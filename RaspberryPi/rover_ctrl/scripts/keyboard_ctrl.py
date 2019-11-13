#!/usr/bin/python2.7
# -*- coding: utf-8 -*-
import rospy
from rover_ctrl.msg import Rov_ctrl
from std_msgs.msg import String
import curses
import os
os.environ.setdefault( 'ESCDELAY', '0' )
import locale
locale.setlocale( locale.LC_ALL, '' )


class Print_manager() :

	def __init__( self, top_gap ) :
		self._top_gap = top_gap
		self._current_line = self._top_gap

	def new_msg( self, msg ) :
		h, w = stdscr.getmaxyx()

		stdscr.addnstr( self._current_line, 0, msg, w - 1 )
		stdscr.clrtoeol()

		self._current_line += 1
		if self._current_line >= h :
			self._current_line = self._top_gap

		if self._current_line > self._top_gap :
			stdscr.addstr( self._current_line, 0, '_'*( w - 1 ), curses.A_DIM )


def callback_info( msg ):
	msg_lines.new_msg( msg.data )


try :

	stdscr = curses.initscr()
	curses.noecho()
	stdscr.keypad( True )
	curses.cbreak()
	curses.curs_set( False )

	msg_lines = Print_manager( 3 )

	pub = rospy.Publisher( 'cmd_nav', Rov_ctrl, queue_size=1 )
	rospy.init_node( 'keyboard_ctrl' )
	sub_info = rospy.Subscriber( 'ctrl_info', String, callback_info )

	repeat_period = rospy.get_param( '~repeat_period', 500 )
	if repeat_period > 0 :
		stdscr.timeout( repeat_period )


	cmd = Rov_ctrl()

	cmd.engaged = False

	speed = 0.0
	speed_inc = 5

	angle = 0.0
	angle_inc = 1
	angle_limit = 40

	torque = 0.0
	torque_inc = 1

	crawling_mode = False

	paused = False


	stdscr.addstr( 0, 0, '[ Engage: SPACE | Speed: UP/DOWN/! | Angle: LEFT/RIGHT/: | Torque: PAGE_UP/PAGE_DOWN/* | CM: ; | Pause/Publish: p ]', curses.A_DIM )

	while not rospy.is_shutdown() :

		if pub.get_num_connections() > 0 :
			stdscr.addstr( 1, 0, '>> connected | ' )
		else :
			stdscr.addstr( 1, 0, 'disconnected | ', curses.A_DIM )

		if cmd.engaged :
			stdscr.addstr( '>> engaged' )
		else :
			stdscr.addstr( 'disengaged', curses.A_DIM )

		stdscr.addstr( ' | Speed: %+.1f mm/s | Angle: %+.f° | Torque: %+.1f N.m' % ( speed, angle, torque ) )

		if crawling_mode :
			stdscr.addstr( ' [crawling mode]' )

		if paused :
			stdscr.addstr( ' [paused]', curses.A_BOLD )
		stdscr.clrtoeol()

		try :
			c = stdscr.getkey()
		except curses.error as what :
			if str( what ) != 'no input' :
				msg_lines.new_msg( 'Curses error: %s' % str( what ) )
			c = 'None'

		if c == ' ' :
			cmd.engaged = not cmd.engaged

		elif c == 'KEY_UP' :
			speed += speed_inc
		elif c == 'KEY_DOWN' :
			speed -= speed_inc
		elif c == '!' :
			speed = 0

		elif c == 'KEY_LEFT' :
			angle -= angle_inc
			if angle < -angle_limit :
				angle = -angle_limit
		elif c == 'KEY_RIGHT' :
			angle += angle_inc
			if angle > angle_limit :
				angle = angle_limit
		elif c == ':' :
			angle = 0

		elif c == 'KEY_PPAGE' :
			torque += torque_inc
		elif c == 'KEY_NPAGE' :
			torque -= torque_inc
		elif c == '*' :
			torque = 0

		elif c == ';' :
			crawling_mode = not crawling_mode

		elif c == 'p' :
			paused = not paused

		elif len( c ) == 1 and ord( c ) == 27 :
			break

		elif repeat_period <= 0 :
			continue

		if c != 'None' and not paused :
			cmd.speed = speed
			cmd.angle = angle
			cmd.torque = torque
			cmd.crawling_mode = crawling_mode

		if repeat_period <= 0 and ( paused or not cmd.engaged ) and c != ' ' :
			continue

		cmd.header.stamp = rospy.Time.now()
		pub.publish( cmd )

except KeyboardInterrupt :
	cmd.engaged = False

	cmd.header.stamp = rospy.Time.now()
	pub.publish( cmd )

#except rospy.ROSInterruptException :
	#pass

finally :
	curses.endwin()
