#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <sched.h>
#include <sys/mman.h>
#include <pthread.h>
#include <math.h>
#include <signal.h>
#include <ros/ros.h>
#include "rover_ctrl/Rov_ctrl.h"
#include "std_msgs/String.h"
#include <sstream>
#include "filters2.hh"


#define PRIORITY 49
#define MAX_SAFE_STACK 8*1024


#define WMC_DEVICE_1 "/dev/ttyACM0"
#define WMC_DEVICE_2 "/dev/ttyACM1"
#define WMC_BAUDRATE B57600
#define RD_BUF_SIZE 1024

#define CMD_ID 0xAD
#define CMD_VEL 0xAA
#define SET_PI 0xA5
#define ID_F 0xAF
#define ID_B 0xAB

#define MC_DEVICE "/dev/serial0"
#define MC_BAUDRATE B921600

#define CMD_ENGAGE 0xAA
#define CMD_DISENGAGE 0x00

#define CMD_MOTOR_1_POS 0xCA
#define CMD_MOTOR_1_POS_SET_KP 0xCB
#define CMD_MOTOR_1_POS_SET_KI 0xCC
#define CMD_MOTOR_1_VEL 0xC5
#define CMD_MOTOR_1_VEL_SET_KP 0xC6
#define CMD_MOTOR_1_VEL_SET_KI 0xC7
#define CMD_MOTOR_1_SET_MAX_VEL 0xC3
#define CMD_MOTOR_1_SET_POS_PREC 0xC4

#define CMD_MOTOR_2_POS 0x3A
#define CMD_MOTOR_2_POS_SET_KP 0x3B
#define CMD_MOTOR_2_POS_SET_KI 0x3C
#define CMD_MOTOR_2_TOR 0x35
#define CMD_MOTOR_2_TOR_SET_KP 0x36
#define CMD_MOTOR_2_TOR_SET_KI 0x37
#define CMD_MOTOR_2_VEL_SET_KP 0x31
#define CMD_MOTOR_2_VEL_SET_KI 0x32
#define CMD_MOTOR_2_SET_MAX_VEL 0x33
#define CMD_MOTOR_2_SET_POS_PREC 0x34

#define FRAME_LENGTH 10
#define N_LAST_FRAMES 2

#define HDR_CJ_ANGLE_RATE 0xCA
#define HDR_SEA_ANGLE_TOR 0x5A

#define MC_TIMEOUT 1. // s
#define CMD_NAV_TIMEOUT 1. // s


#define DEG_TO_RAD 1.7453292519943295e-2

#define WBASE 580 // mm
#define WTRACK 610 // mm
#define WRADIUS 105 // mm

#define WSPEED_MAX 7.6365 // rad/s


#define LOOP_FREQ 50 // Hz


ros::Publisher info_ctrl_pub;


void init_rt()
{
	struct sched_param param;

	// Declare ourself as a real time task
	param.sched_priority = PRIORITY;
	if( sched_setscheduler( 0, SCHED_FIFO, &param ) == -1 )
	{
		perror( "sched_setscheduler failed" );
		exit( -2 );
	}

	// Lock memory
	if( mlockall( MCL_CURRENT | MCL_FUTURE ) == -1 )
	{
		perror( "mlockall failed" );
		exit( -2 );
	}

	// Pre-fault our stack
	unsigned char dummy[MAX_SAFE_STACK];
	memset( dummy, 0, MAX_SAFE_STACK );
}


int open_uart( const char *device_name, speed_t baudrate )
{
	struct termios options;
	int fd;
	 
	fd = open( device_name, O_RDWR | O_NOCTTY | O_NDELAY );

	if ( fd == -1 )
	{
		fprintf( stderr, "Can't open port %s: %s\n", device_name, strerror( errno ) );
		return -1;
	}

	fcntl( fd, F_SETFL, 0 );

	// Get the current options for the port
	tcgetattr( fd, &options );

	// Set the baud rates to 921600
	cfsetispeed( &options, baudrate );
	cfsetospeed( &options, baudrate );

	// 8 Bit, No Parity, 1 Stop Bit
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;

	//Enable the receiver and set local mode
	options.c_cflag |= ( CLOCAL | CREAD );

	// RAW mode
	options.c_lflag &= ~( ICANON | ECHO | ECHOE | ISIG );
	options.c_oflag &= ~OPOST;
	options.c_iflag &= ~( IXON | IXOFF );
	options.c_iflag &= IGNCR;

	// Set the new options for the port
	tcsetattr( fd, TCSANOW, &options );

	return fd;
}


bool send_cmd1( const int fd, const uint8_t cmd, const float val=0 )
{
	union raw_float {
		float fval;
		uint8_t bytes[4];
	} data;
	data.fval = val;

	uint8_t buffer[6];
	buffer[0] = cmd;
	buffer[5] = cmd;
	for ( int i = 0 ; i < 4 ; i++ )
	{
		buffer[i+1] = data.bytes[i];
		buffer[5] ^= data.bytes[i];
	}

	if ( write( fd, buffer, 6 ) != 6 )
	{
		fprintf( stderr, "Failed to send [%#04x %f] to file descriptor %d: %s\n", cmd, val, fd, strerror( errno ) );
		return false;
	}

	return true;
}


bool send_cmd2( const int fd, const uint8_t cmd, const float val1=0, const float val2=0 )
{
	union raw_floats {
		float floats[2];
		uint8_t bytes[8];
	} data;
	data.floats[0] = val1;
	data.floats[1] = val2;

	uint8_t buffer[10];
	buffer[0] = cmd;
	buffer[9] = cmd;
	for ( int i = 0 ; i < 8 ; i++ )
	{
		buffer[i+1] = data.bytes[i];
		buffer[9] ^= data.bytes[i];
	}

	if ( write( fd, buffer, 10 ) != 10 )
	{
		fprintf( stderr, "Failed to send [%#04x %f %f] to file descriptor %d: %s\n", cmd, val1, val2, fd, strerror( errno ) );
		return false;
	}

	return true;
}


bool open_and_identify_wmc( const std::string devices[2], speed_t baudrate, int& F_fd, int& B_fd )
{
	int fd[2];
	uint8_t id[2] = { 0 };
	for ( int i = 0 ; i < 2 ; i++ )
	{
		if ( ( fd[i] = open_uart( devices[i].c_str(), baudrate ) ) == -1 )
			return false;

		sleep( 2 );
		ioctl( fd[i], TCFLSH, 2 );

		if ( ! send_cmd2( fd[i], CMD_ID ) )
			return false;

		printf( "Listening %s for ID...\n", devices[i].c_str() );
		do
		{
			read( fd[i], id + i, 1 );
			printf( "Read %#04x (%c)\n", id[i], id[i] );
		} while ( id[i] != ID_F && id[i] != ID_B );
	}

	if ( id[0] == ID_F && id[1] == ID_B )
	{
		F_fd = fd[0];
		B_fd = fd[1];
	}
	else if ( id[1] == ID_F && id[0] == ID_B )
	{
		F_fd = fd[1];
		B_fd = fd[0];
	}
	else
	{
		fprintf( stderr, "Couldn't identify both WMC.\n" );
		return false;
	}

	printf( "Both WMC successfully identified: F_fd = %d and B_fd = %d.\n", F_fd, B_fd );

	return true;
}


void read_from_wmc( const int F_fd, const int B_fd )
{
	static char buffer[2][RD_BUF_SIZE];
	static char ros_info_buf[RD_BUF_SIZE];
	static int rd_pos[2];

	for ( int i = 0 ; i < 2 ; i++ )
	{
		int rd;
		if ( ioctl( ( i == 0 ? F_fd : B_fd ), FIONREAD, &rd ) == -1 )
			fprintf( stderr, "Failed to get the amount of available data from %s: %s\n", ( i == 0 ? "Front WMC" : "Back  WMC" ), strerror( errno ) );
		else if ( rd == 0 )
			continue;

		rd = read( ( i == 0 ? F_fd : B_fd ), buffer[i] + rd_pos[i], RD_BUF_SIZE - rd_pos[i] - 1 );

		if ( rd == -1 )
		{
			fprintf( stderr, "Failed to read from %s: %s\n", ( i == 0 ? "Front WMC" : "Back  WMC" ), strerror( errno ) );
			continue;
		}
		else if ( rd == RD_BUF_SIZE - rd_pos[i] - 1 )
			ROS_WARN( "%s is sending to much data in a row!", ( i == 0 ? "Front WMC" : "Back  WMC" ) );

		int j;
		for ( j = rd_pos[i] ; j < rd_pos[i] + rd ; j++ )
		{
			if ( buffer[i][j] == '\n' || j == RD_BUF_SIZE - 2 )
			{
				int isend = ( j == RD_BUF_SIZE - 2 ? 1 : 0 );
				int iscarret = ( j > 0 && buffer[i][j-1] == '\r' ? 1 : 0 );

				memmove( ros_info_buf, buffer[i], j + isend - iscarret );
				ros_info_buf[j + isend - iscarret] = '\0';
				if ( ros_info_buf[0] == '!' )
					ROS_WARN( "Received from %s: [ %s ]", ( i == 0 ? "Front WMC" : "Back  WMC" ), ros_info_buf );
				else
					ROS_INFO( "Received from %s: [ %s ]", ( i == 0 ? "Front WMC" : "Back  WMC" ), ros_info_buf );

				// Send the message on the str_info topic:
				std_msgs::String msg;
				msg.data = std::string( ros_info_buf );
				info_ctrl_pub.publish( msg );

				rd = rd_pos[i] + rd - j - 1;
				memmove( buffer[i], buffer[i] + j + 1, rd );
				rd_pos[i] = 0;
				j = -1;
			}
		}
		rd_pos[i] = j;
	}
}


float cj_angle = 0;
float cj_rate = 0;
float sea_angle = 0;
float sea_torque = 0;


int checksum( const uint8_t* const buffer, const int buf_pos )
{
	uint8_t cs = 0;
	for ( int i = 0 ; i < FRAME_LENGTH ; i++ )
		cs ^= buffer[(buf_pos+i)%FRAME_LENGTH];
	if ( cs != 0 )
		ROS_WARN( "Checksum failed for header %#04x.", buffer[buf_pos%FRAME_LENGTH] );
	return cs;
}

int read_from_mc( const int fd )
{
	uint8_t rd_buf[FRAME_LENGTH], discard_buf[1024];

	union {
		uint8_t bytes[4];
		float floats[2];
	} data;

	int available;
	int rd = 0;

	//if ( Serial.available() < FRAME_LENGTH )
		//return 0;
	if ( ioctl( fd, FIONREAD, &available ) == -1 )
		fprintf( stderr, "Failed to get the amount of available data from MC: %s\n", strerror( errno ) );
	else if ( available < FRAME_LENGTH )
		return 0;

	//while ( Serial.available() > FRAME_LENGTH*N_LAST_FRAMES )
		//Serial.read();
	while ( available - FRAME_LENGTH*N_LAST_FRAMES )
	{
		rd = read( fd, discard_buf, std::min( 1024, available - FRAME_LENGTH*N_LAST_FRAMES ) );
		available -= rd;
	}

	//for ( int i = 0 ; i < FRAME_LENGTH - 1 ; i++ )
		//rd_buf[i] = Serial.read();
	rd = read( fd, rd_buf, FRAME_LENGTH - 1 );

	int buf_pos = 0;
	//while ( Serial.available() >= 1 )
	while ( ioctl( fd, FIONREAD, &available ) != -1 && available >= 1 )
	{
		//rd_buf[(buf_pos+FRAME_LENGTH-1)%FRAME_LENGTH] = Serial.read();
		rd = read( fd, rd_buf + ( buf_pos + FRAME_LENGTH - 1 )%FRAME_LENGTH, 1 );

		switch ( rd_buf[buf_pos%FRAME_LENGTH] )
		{
			case HDR_CJ_ANGLE_RATE :
				
				if ( checksum( rd_buf, buf_pos ) == 0 )
				{
					for ( int i = 0 ; i < FRAME_LENGTH - 2 ; i++ )
						data.bytes[i] = rd_buf[(buf_pos+1+i)%FRAME_LENGTH];

					cj_angle = data.floats[0];
					cj_rate = data.floats[1];

					return 1;
				}
				break;

			case HDR_SEA_ANGLE_TOR :
				
				if ( checksum( rd_buf, buf_pos ) == 0 )
				{
					for ( int i = 0 ; i < FRAME_LENGTH - 2 ; i++ )
						data.bytes[i] = rd_buf[(buf_pos+1+i)%FRAME_LENGTH];

					sea_angle = data.floats[0];
					sea_torque = data.floats[1];

					return 2;
				}
				break;
		}

		buf_pos++;
	}

	ROS_WARN( "Couldn't identify the frame sent by MC!" );

	return -1;
}


int mc_fd, wmc_F_fd, wmc_B_fd;


void disengage_all()
{
	send_cmd2( wmc_F_fd, CMD_VEL, 0, 0 );
	send_cmd2( wmc_B_fd, CMD_VEL, 0, 0 );

	send_cmd1( mc_fd, CMD_DISENGAGE, 0 );
}


void stop_procedure( int sig )
{
	disengage_all();
	exit( 0 );
}


bool engaged_cmd = false;
float speed_cmd = 0;
float angle_cmd = 0;
float torque_cmd = 0;
bool crawling_mode_cmd = false;
ros::Time last_update_cmd_nav;


void cmd_rcv_Callback( const rover_ctrl::Rov_ctrl::ConstPtr& msg )
{
	if ( msg->engaged != engaged_cmd )
	{
		if ( ! msg->engaged )
		{
			disengage_all();
		}
		else
			send_cmd1( mc_fd, CMD_ENGAGE, 0 );
	}
	engaged_cmd = msg->engaged;
	speed_cmd = msg->speed;
	angle_cmd = msg->angle;
	torque_cmd = msg->torque;
	crawling_mode_cmd = msg->crawling_mode;

	last_update_cmd_nav = msg->header.stamp;

	ROS_INFO( "New command received: [ %s %f %f %f %s ]", ( engaged_cmd ? "true" : "false" ),
	          speed_cmd, angle_cmd, torque_cmd, ( crawling_mode_cmd ? "true" : "false" ) );
}


int main( int argc, char **argv )
{
	//init_rt();

	ros::init( argc, argv, "nav_node" );

	ros::NodeHandle nh;

	std::string wmc_devices[2], mc_device;
	int wmc_baudrate, mc_baudrate;
	float loop_freq;
	nh.param<std::string>( "wmc_device_1", wmc_devices[0], std::string( WMC_DEVICE_1 ) );
	nh.param<std::string>( "wmc_device_2", wmc_devices[1], std::string( WMC_DEVICE_2 ) );
	nh.param( "wmc_baudrate", wmc_baudrate, WMC_BAUDRATE );
	nh.param<std::string>( "mc_device", mc_device, std::string( MC_DEVICE ) );
	nh.param( "mc_baudrate", mc_baudrate, MC_BAUDRATE );
	nh.param( "loop_rate", loop_freq, (float) LOOP_FREQ );

	ros::Rate loop_rate( loop_freq );

	if ( signal( SIGINT, stop_procedure ) == SIG_ERR )
	{
		perror( "Failed to attribute the stop procedure to the interruption signal" );
		return -2;
	}

	if ( ( mc_fd = open_uart( mc_device.c_str(), mc_baudrate ) ) == -1 )
		return -1;
	printf( "Communication successfully set up with MC with file descriptor number %d.\n", mc_fd );

	if ( ! open_and_identify_wmc( wmc_devices, wmc_baudrate, wmc_F_fd, wmc_B_fd ) )
		return -1;

	ros::Subscriber sub = nh.subscribe( "cmd_nav", 1, cmd_rcv_Callback );

	info_ctrl_pub = nh.advertise<std_msgs::String>( "ctrl_info", 50 );
	std_msgs::String msg;


	// Modification of MC controller gains:

	//send_cmd1( mc_fd, CMD_MOTOR_1_POS_SET_KP, 2 );
	//send_cmd1( mc_fd, CMD_MOTOR_1_VEL_SET_KP, 1 );
	//send_cmd1( mc_fd, CMD_MOTOR_1_VEL_SET_KI, 1 );
	//send_cmd1( mc_fd, CMD_MOTOR_1_SET_MAX_VEL, 5 );
	//send_cmd1( mc_fd, CMD_MOTOR_1_SET_POS_PREC, 0.5 );

	//sleep( 1 );

	//send_cmd1( mc_fd, CMD_MOTOR_2_POS_SET_KP, 3 );
	//send_cmd1( mc_fd, CMD_MOTOR_2_TOR_SET_KP, 30 );
	//send_cmd1( mc_fd, CMD_MOTOR_2_VEL_SET_KP, 1 );
	//send_cmd1( mc_fd, CMD_MOTOR_2_VEL_SET_KI, 1 );
	//send_cmd1( mc_fd, CMD_MOTOR_2_SET_MAX_VEL, 30 );
	// Fpr a smoother position control:
	//send_cmd1( mc_fd, CMD_MOTOR_2_SET_MAX_VEL, 20 );
	//send_cmd1( mc_fd, CMD_MOTOR_2_SET_POS_PREC, 0.5 );


	// Recursive filter in order to smoothen speed variations:
	float speed_cmd_filtered = speed_cmd;
	filters::LP_second_order<float> speed_cmd_filter;
	speed_cmd_filter.init_bilinear( 1./LOOP_FREQ, 2*M_PI*1, 0.5, &speed_cmd, &speed_cmd_filtered );


	float beta, dbeta_dt;
	float omega_steer, omega_turn;
	float w_cmd[4] = { 0 };

	float _wradius = 1./WRADIUS;
	float wtrack_wbase = (float) WTRACK/WBASE;

	ros::Duration mc_timeout( MC_TIMEOUT );
	ros::Time last_update_from_mc = ros::Time::now();

	ros::Duration cmd_nav_timeout( CMD_NAV_TIMEOUT );
	bool cmd_nav_connected = false;

	ros::Time current_time;
	ros::Time last_time = ros::Time::now();
	double dt;

	while ( ros::ok() )
	{
		ros::spinOnce();

		speed_cmd_filter.update();

		// Check the reception of navigation commands:
		bool disconnection = sub.getNumPublishers() == 0 || ros::Time::now() - last_update_cmd_nav >= cmd_nav_timeout;
		if ( cmd_nav_connected && disconnection )
		{
			if ( engaged_cmd )
				disengage_all();
			cmd_nav_connected = false;
		}
		else if ( ! cmd_nav_connected && ! disconnection )
		{
			if ( engaged_cmd )
				send_cmd1( mc_fd, CMD_ENGAGE, 0 );
			cmd_nav_connected = true;
		}

		// Check the communication with MC:
		while ( read_from_mc( mc_fd ) > 0 ) { last_update_from_mc = ros::Time::now(); }
		if ( ros::Time::now() - last_update_from_mc >= mc_timeout )
		{
			fprintf( stderr, "Haven't received any update from MC for %fs!\n", MC_TIMEOUT );
			stop_procedure( 0 );
		}

		
		// -----------------------------------------------
		// Computation of the wheel speed synchronisation:

		beta = cj_angle*DEG_TO_RAD;
		dbeta_dt = cj_rate*DEG_TO_RAD;
		
		//for ( int i = 0 ; i < 4 ; i++ )
		//{
			//omega_steer = ( i/2 ? -1 : 1 )*( -WBASE*tan( beta*0.5 ) + ( i%2 ? -1 : 1 )*WTRACK )*dbeta_dt*0.25*_wradius;
			//omega_turn = ( i%2 ? -1 : 1 )*wtrack_wbase*tan( beta*0.5 )*speed_cmd_filtered*_wradius;
			//w_cmd[i] = speed_cmd_filtered*_wradius + omega_steer + omega_turn;

			// Watch wheel speed limit:
			//if ( w_cmd[i] > WSPEED_MAX )
			//{
				//std::stringstream ss;
				//ss << "<!> Speed exceeded for wheel " << i << " (" << w_cmd[i] << "/" << WSPEED_MAX << ")";
				//msg.data = ss.str();
				//info_ctrl_pub.publish( msg );
			//}
		//}

		float diff[4];
		float _1diff[4];
		float trans[4];
		float min_speed = 0;
		float final_speed;
		for ( int i = 0 ; i < 4 ; i++ )
		{
			diff[i] = ( i%2 ? -1 : 1 )*wtrack_wbase*tan( beta*0.5 );
			_1diff[i] = 1/( 1 + diff[i] );
			trans[i] = ( i/2 ? -1 : 1 )*( -WBASE*tan( beta*0.5 ) + ( i%2 ? -1 : 1 )*WTRACK )*dbeta_dt*0.25;

			if ( crawling_mode_cmd )
			{
				// Look for the minimal robot speed that prevents any wheel from going backward:
				if ( speed_cmd_filtered >= 0 )
					min_speed = std::max( min_speed, -trans[i]*_1diff[i] );
				else
					min_speed = std::min( min_speed, -trans[i]*_1diff[i] );
			}
		}
		// Assert that the robot speed is high enough:
		if ( speed_cmd_filtered >= 0 )
			final_speed = std::max( speed_cmd_filtered, min_speed );
		else
			final_speed = std::min( speed_cmd_filtered, min_speed );

		// Reduce the robot speed according to the wheel speed limit:
		for ( int i = 0 ; i < 4 ; i++ )
			if ( speed_cmd_filtered >= 0 )
				final_speed = std::min( final_speed, ( (float) WSPEED_MAX*WRADIUS - trans[i] )*_1diff[i] );
			else
				final_speed = std::max( final_speed, ( (float) -WSPEED_MAX*WRADIUS - trans[i] )*_1diff[i] );

		// Compute corresponding the speed for each wheel:
		for ( int i = 0 ; i < 4 ; i++ )
			w_cmd[i] = ( final_speed*( 1 + diff[i] ) + trans[i] )*_wradius;

		// -----------------------------------------------


		// Send the commands:
		if ( engaged_cmd && cmd_nav_connected )
		{
			send_cmd2( wmc_F_fd, CMD_VEL, w_cmd[0], w_cmd[1] );
			send_cmd2( wmc_B_fd, CMD_VEL, w_cmd[2], w_cmd[3] );

			send_cmd1( mc_fd, CMD_MOTOR_1_POS, angle_cmd );
			//send_cmd1( mc_fd, CMD_MOTOR_1_VEL, angle_cmd );

			//send_cmd1( mc_fd, CMD_MOTOR_2_POS, torque_cmd );
			send_cmd1( mc_fd, CMD_MOTOR_2_TOR, torque_cmd );
		}

		//printf( "angle_A: %f | vel_A: %f | b_angle: %f | b_torque: %f | loop duration: %f\n", cj_angle, cj_rate, sea_angle, sea_torque, dt );
		//printf( "angle_A: %f | vel_A: %f | angle_B: %f | angle_C: %f | loop duration: %f\n", cj_angle, cj_rate, sea_angle, sea_torque, dt );
		//printf( "err1vel: %f | interr1vel: %f | pct_1: %f | pct_2: %f | loop duration: %f\n", cj_angle, cj_rate, sea_angle, sea_torque, dt );
		//printf( "err1vel: %f | interr1vel: %f | vel_A: %f | filtered_vel: %f\n", cj_angle, cj_rate, sea_angle, sea_torque );
		//printf( "%f %f %f %f\n", w_cmd[0], w_cmd[1], w_cmd[2], w_cmd[3] );

		read_from_wmc( wmc_F_fd, wmc_B_fd );

		loop_rate.sleep();

		current_time = ros::Time::now();
		dt = ( current_time - last_time ).toSec();
		last_time = current_time;
	}

	return 0;
}
