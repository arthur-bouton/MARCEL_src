#!/usr/bin/python2.7
# -*- coding: utf-8 -*-
import rospy
from rover_ctrl.msg import Rov_ctrl
import curses
import os
os.environ.setdefault( 'ESCDELAY', '0' )


try :

	stdscr = curses.initscr()
	curses.noecho()
	stdscr.keypad( True )
	curses.cbreak()
	curses.curs_set( False )

	pub = rospy.Publisher( 'nav_ctrl', Rov_ctrl, queue_size=1 )
	rospy.init_node( 'keyboard_ctrl' )

	while not rospy.is_shutdown() :

		speed = 0.0
		speed_inc = 5

		angle = 0.0
		angle_inc = 1
		angle_limit = 50

		torque = 0.0
		torque_inc = 5

		crawling_mode = False

		pause = False

		stdscr.addstr( 0, 0, '[ Speed: UP/DOWN/SPACE | Angle: LEFT/RIGHT/! | Torque: PAGE_UP/PAGE_DOWN/Enter | CM: ; | Pause: p ]', curses.A_DIM )

		cmd = Rov_ctrl()

		while True :

			if not pause :
				cmd.speed = speed
				cmd.rate = angle
				cmd.torque = torque
				cmd.crawling_mode = crawling_mode

				pub.publish( cmd )

			stdscr.addstr( 1, 0, 'Speed: %+.1f mm/s | Angle: %+.f° | Torque: %+.1f N.m' % ( speed, angle, torque ) )
			if crawling_mode :
				stdscr.addstr( ' [crawling mode]' )
			if pause :
				stdscr.addstr( ' [pause]' )
			stdscr.clrtoeol()
			#stdscr.refresh()

			c = stdscr.getkey()

			if c == 'KEY_UP' :
				speed += speed_inc
			elif c == 'KEY_DOWN' :
				speed -= speed_inc
			elif c == ' ' :
				speed = 0

			elif c == 'KEY_LEFT' :
				angle -= angle_inc
				if angle < -angle_limit :
					angle = -angle_limit
			elif c == 'KEY_RIGHT' :
				angle += angle_inc
				if angle > angle_limit :
					angle = angle_limit
			elif c == '!' :
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
				pause = not pause

			elif len( c ) == 1 and ord( c ) == 27 :
				#speed = 0
				break

			else :
				continue

except curses.error as what :
	if str( what ) != 'no input' :
		curses.endwin()
		print( 'Curses error: %s' % str( what ) )
except KeyboardInterrupt :
	pass
except rospy.ROSInterruptException :
	pass

finally :
	curses.endwin()
