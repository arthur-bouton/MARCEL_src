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


steer_max = 15.
torque_max = 20.


class Print_manager() :

	def __init__( self, scr, top_gap ) :
		self._top_gap = top_gap
		self._current_line = self._top_gap

	def new_msg( self, msg ) :
		h, w = scr.getmaxyx()

		scr.addnstr( self._current_line, 0, msg, w - 1 )
		scr.clrtoeol()

		self._current_line += 1
		if self._current_line >= h :
			self._current_line = self._top_gap

		if self._current_line > self._top_gap :
			scr.addstr( self._current_line, 0, '_'*( w - 1 ), curses.A_DIM )


def callback_nav_info( msg ):
	msg_lines.new_msg( msg.data )


def print_bar( scr, line, value, value_max, right_space=0 ) :

	h, w = scr.getmaxyx()
	l_half_gauge = int( w - right_space )/2
	l_bar = min( abs( int( value/value_max*l_half_gauge ) ), l_half_gauge )

	pieces = []
	pieces.append( ' '*l_half_gauge )
	pieces.append( '|' )
	pieces.append( '█'*l_bar )
	pieces.append( ' '*( l_half_gauge - l_bar ) )

	if value < 0 :
		pieces.reverse()

	scr.move( line, 0 )
	for piece in pieces :
		scr.addstr( piece )


def callback_cmd_info( msg ):
	print_bar( stdscr, 3, msg.steer, steer_max, 22 )
	stdscr.addstr( '| Steering: %+5.1f°' % steer )

	print_bar( stdscr, 4, msg.torque, torque_max, 22 )
	stdscr.addstr( '| Torque:   %+5.1f N.m' % torque )

	stdscr.refresh()


try :

	stdscr = curses.initscr()
	curses.noecho()
	stdscr.keypad( True )
	curses.cbreak()
	curses.curs_set( False )

	msg_lines = Print_manager( stdscr, 5 )

	pub = rospy.Publisher( 'cmd_ctrl', Rov_ctrl, queue_size=1 )
	rospy.init_node( 'keyboard_cmd' )
	sub_cmd_info = rospy.Subscriber( 'nav_ctrl', String, callback_cmd_info )
	sub_nav_info = rospy.Subscriber( 'nav_info_string', String, callback_nav_info )

	repeat_period = rospy.get_param( '~repeat_period', 500 )
	if repeat_period > 0 :
		stdscr.timeout( repeat_period )


	cmd = Rov_ctrl()

	cmd.engaged = False

	speed = 0.0
	speed_inc = 5

	cmd.steer = 0.0
	cmd.torque = 0.0
	cmd.rate_mode = False
	cmd.crawling_mode = False

	paused = False


	stdscr.addstr( 0, 0, '[ Engage: SPACE | Speed: UP/DOWN/! | Pause/Publish: p ]', curses.A_DIM )

	while not rospy.is_shutdown() :

		if pub.get_num_connections() > 0 :
			stdscr.addstr( 1, 0, '>> connected | ' )
		else :
			stdscr.addstr( 1, 0, 'disconnected | ', curses.A_DIM )

		if cmd.engaged :
			stdscr.addstr( '>> engaged' )
		else :
			stdscr.addstr( 'disengaged', curses.A_DIM )

		stdscr.addstr( ' | Speed: %+.1f mm/s' % speed )

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

		elif c == 'p' :
			paused = not paused

		elif len( c ) == 1 and ord( c ) == 27 :
			break

		elif repeat_period <= 0 :
			continue

		if c != 'None' and not paused :
			cmd.speed = speed

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
