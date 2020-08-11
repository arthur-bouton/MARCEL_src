#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <functional> // std::bind
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "geometry_msgs/WrenchStamped.h"

#include "robotiq_ft_sensor/sensor_accessor.h" // from https://github.com/ros-industrial/robotiq.git
#include "rover_ctrl/Joints_info.h"
#include "rover_ctrl/Rov_ctrl.h"

#include "ModelTree/cpp/model_tree.hh" // git clone https://github.com/Bouty92/ModelTree


#define LMT_YAML_FILE_PATH_1 "/home/ubuntu/MARCEL_src/RaspberryPi/rover_ctrl/tree_params_1.yaml"
#define LMT_YAML_FILE_PATH_2 "/home/ubuntu/MARCEL_src/RaspberryPi/rover_ctrl/tree_params_2.yaml"


#define FT_SERIAL_NUMBER_FRONT "  F-31951"
#define FT_SERIAL_NUMBER_REAR  "  F-31952"

#define CALIB_NB_SAMPLES 10
#define CALIB_DURATION 2 // s

// Inital reference values for the vertical forces to be measured by the FT sensors:
#define FRONT_FZ 65 // N
#define REAR_FZ  55 // N


#define RAD_TO_DEG 57.29577951308232


#define LOOP_FREQ 2 // Hz

#define CMD_CTRL_TIMEOUT 1. // s

#define USE_ONBOARD_INCLINOMETER

#define STEERING_MAX_VEL  (float) 15 // °/s
#define BOGGIE_MAX_TORQUE (float) 20 // N.m


//===============================================================//
//------------------------------SPI------------------------------//
//===============================================================//

#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include "gpio_access.h"


#define INC_CSB 22
#define INC_RDAX 0x10
#define INC_RDAY 0x11

#define INC_OFFSET_X 0.8 // °
#define INC_OFFSET_Y 2.0 // °


int setup_spi( std::vector<unsigned int> csb_list, const char* device="/dev/spidev0.0" )
{
	// Initialize chip select lines:
	setup_gpio_address();
	for ( int i=0 ; i < csb_list.size() ; i++ )
	{
		OUT_GPIO( csb_list[i] );
		GPIO_SET = 1 << csb_list[i];
	}


	// SPI configuration:
	unsigned int speed = 5e5;
	unsigned char spi_mode = SPI_MODE_0;
	//SPI_MODE_0 (0,0) 	CPOL = 0, CPHA = 0, Clock idle low, data is clocked in on rising edge, output data (change) on falling edge
	//SPI_MODE_1 (0,1) 	CPOL = 0, CPHA = 1, Clock idle low, data is clocked in on falling edge, output data (change) on rising edge
	//SPI_MODE_2 (1,0) 	CPOL = 1, CPHA = 0, Clock idle high, data is clocked in on falling edge, output data (change) on rising edge
	//SPI_MODE_3 (1,1) 	CPOL = 1, CPHA = 1, Clock idle high, data is clocked in on rising, edge output data (change) on falling edge

	int spi_fd = open( device, O_RDWR );
	if ( spi_fd < 0 )
	{
		ROS_FATAL( "Can't open spidev: %s", strerror( errno ) );
		exit( -1 );
	}
	if ( ioctl( spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed ) != 0 )
	{
		ROS_FATAL( "ioctl couldn't set writing speed: %s", strerror( errno ) );
		exit( -1 );
	}
	if ( ioctl( spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed ) != 0 )
	{
		ROS_FATAL( "ioctl couldn't set reading speed: %s", strerror( errno ) );
		exit( -1 );
	}
	if ( ioctl( spi_fd, SPI_IOC_WR_MODE, &spi_mode ) != 0 )
	{
		ROS_FATAL( "ioctl couldn't set writing mode: %s", strerror( errno ) );
		exit( -1 );
	}

	if ( ioctl( spi_fd, SPI_IOC_RD_MODE, &spi_mode ) != 0 )
	{
		ROS_FATAL( "ioctl couldn't set reading mode: %s", strerror( errno ) );
		exit( -1 );
	}

	return spi_fd;
}


int16_t read_spi( int fd, unsigned int csb, unsigned char cmd )
{
	unsigned char byte[2];

	GPIO_CLR = 1 << INC_CSB;

	usleep( 150 );

	if ( write( fd, &cmd, 1 ) != 1 )
	{
		ROS_ERROR( "Failed to write on SPI: %s", strerror( errno ) );
		exit( -2 );
	}

	read( fd, byte, 2 );

	int16_t acc = ( ( ( byte[0] << 8 ) + byte[1] ) >> 5 ) - 1024;

	GPIO_SET = 1 << INC_CSB;

	return acc;
}


void get_inclinaisons( const int fd, float& angle_x, float& angle_y )
{
	// Get the accelerations from the inclinometer:
	int16_t acc_x = read_spi( fd, INC_CSB, INC_RDAX );
	usleep( 150 );
	int16_t acc_y = read_spi( fd, INC_CSB, INC_RDAY );

	// Convert the accelerations into tilt angles:
	angle_x = -asin( acc_y/819. )*RAD_TO_DEG - INC_OFFSET_X;
	angle_y = -asin( acc_x/819. )*RAD_TO_DEG - INC_OFFSET_Y;
}


//================================================================//
//------------------------------UART------------------------------//
//================================================================//

#include <fcntl.h>
#include <termios.h>


#define COMPASS_DEVICE "/dev/ttyUSB0"
#define COMPASS_BAUDRATE B9600
#define COMPASS_GET_BEARING_16_BIT 0x13
#define COMPASS_GET_ALL 0x23
#define COMPASS_GET_PITCH 0x14
#define COMPASS_GET_ROLL 0x15


int open_uart( const char *device_name, speed_t baudrate )
{
	struct termios options;
	int fd;
	 
	fd = open( device_name, O_RDWR | O_NOCTTY | O_NDELAY );

	if ( fd == -1 )
	{
		ROS_FATAL( "Can't open the port %s: %s", device_name, strerror( errno ) );
		return -1;
	}

	fcntl( fd, F_SETFL, 0 );

	// Get the current options for the port:
	tcgetattr( fd, &options );

	// Set the baud rate:
	cfsetispeed( &options, baudrate );
	cfsetospeed( &options, baudrate );

	// 8 bits, no parity, 2 stop bits:
	options.c_cflag &= ~PARENB;
	//options.c_cflag &= ~CSTOPB; // 1 stop bit
	options.c_cflag |= CSTOPB; // 2 stop bits
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;

	// Enable the receiver and set local mode:
	options.c_cflag |= ( CLOCAL | CREAD );

	// RAW mode:
	options.c_lflag &= ~( ICANON | ECHO | ECHOE | ISIG );
	options.c_oflag &= ~OPOST;
	options.c_iflag &= ~( IXON | IXOFF );
	options.c_iflag &= IGNCR;

	// Set the new options for the port:
	tcsetattr( fd, TCSANOW, &options );

	return fd;
}


bool compass_get_bearing( const int fd, float& angle )
{
	uint8_t cmd = COMPASS_GET_BEARING_16_BIT;

	if ( write( fd, &cmd, 1 ) != 1 )
	{
		ROS_ERROR( "Failed to request the bearing from the compass: %s", strerror( errno ) );
		return false;
	}

	uint16_t bearing;

	if ( read( fd, &bearing, 2 ) != 2 )
	{
		ROS_ERROR( "Failed to receive the bearing from the compass: %s", strerror( errno ) );
		return false;
	}

	// High byte first:
	bearing = ( bearing >> 8 ) | ( ( bearing & 0xFF ) << 8 );

	if ( bearing > 3599 )
	{
		ROS_ERROR( "Wrong bearing value received from the compass: %i", bearing );
		return false;
	}

	angle = bearing*0.1;

	return true;
}


bool compass_get_all( const int fd, float& bearing, short& roll, short& pitch )
{
	uint8_t cmd = COMPASS_GET_ALL;

	if ( write( fd, &cmd, 1 ) != 1 )
	{
		ROS_ERROR( "Failed to request the data from the compass: %s", strerror( errno ) );
		return false;
	}

	struct {
	//struct __attribute__((__packed__)) {
		uint16_t bearing;
		int8_t pitch;
		int8_t roll;
	} data;

	if ( read( fd, &data, 4 ) != 4 )
	{
		ROS_ERROR( "Failed to receive the data from the compass: %s", strerror( errno ) );
		return false;
	}

	// High byte first:
	data.bearing = ( data.bearing >> 8 ) | ( ( data.bearing & 0xFF ) << 8 );

	bool data_all_valid = true;

	if ( data.bearing > 3599 )
	{
		ROS_ERROR( "Wrong bearing value received from the compass: %i", data.bearing );
		data_all_valid = false;
	}
	else
		bearing = data.bearing*0.1;

	if ( data.pitch > 90 || data.pitch < -90 )
	{
		ROS_ERROR( "Wrong pitch value received from the compass: %i", data.pitch );
		data_all_valid = false;
	}
	else
		pitch = data.pitch;

	if ( data.roll > 90 || data.roll < -90 )
	{
		ROS_ERROR( "Wrong roll value received from the compass: %i", data.roll );
		data_all_valid = false;
	}
	else
		roll = data.roll;

	return data_all_valid;
}

//----------------------------------------------------------------//
//================================================================//


double ft_time[2] = { 0 };
float ft_list[4][3] = { 0 };
std::vector<std::vector<float>> ft_offsets = { { 0, 0, FRONT_FZ }, { 0, 0, 0 }, { 0, 0, REAR_FZ }, { 0, 0, 0 } };
bool ft_dirty[2] = { false };

void wrench_rcv_Callback( const geometry_msgs::WrenchStamped::ConstPtr& msg, int id )
{
	ft_time[id] = msg->header.stamp.toSec();
	ft_list[id*2][0] = msg->wrench.force.y - ft_offsets[id*2][0];
	ft_list[id*2][1] = msg->wrench.force.x - ft_offsets[id*2][1];
	ft_list[id*2][2] = -msg->wrench.force.z - ft_offsets[id*2][2];
	ft_list[id*2+1][0] = msg->wrench.torque.y - ft_offsets[id*2+1][0];
	ft_list[id*2+1][1] = msg->wrench.torque.x - ft_offsets[id*2+1][1];
	ft_list[id*2+1][2] = -msg->wrench.torque.z - ft_offsets[id*2+1][2];

	ft_dirty[id] = true;
}


double joints_info_time = 0;
float cj_angle = 0;
float sea_angle = 0;
bool joints_info_dirty = false;

void joints_info_rcv_Callback( const rover_ctrl::Joints_info::ConstPtr& msg )
{
	joints_info_time = msg->header.stamp.toSec();
	cj_angle = msg->cj_angle;
	sea_angle = msg->sea_angle;

	joints_info_dirty = true;
}


ros::Publisher nav_ctrl_pub;
rover_ctrl::Rov_ctrl nav_ctrl_msg;
ros::Time last_update_cmd_ctrl;

void cmd_ctrl_rcv_Callback( const rover_ctrl::Rov_ctrl::ConstPtr& msg )
{
	nav_ctrl_msg.header.stamp = msg->header.stamp;
	nav_ctrl_msg.engaged = msg->engaged;
	nav_ctrl_msg.speed = msg->speed;
	nav_ctrl_pub.publish( nav_ctrl_msg );

	last_update_cmd_ctrl = msg->header.stamp;
}


int main( int argc, char **argv )
{
	ros::init( argc, argv, "cmd_mt" );

	ros::NodeHandle nh;

	float loop_freq;
	nh.param( "loop_rate", loop_freq, (float) LOOP_FREQ );
	ros::Rate loop_rate( loop_freq );

	std::string ft_serial_numbers[2];
	nh.param<std::string>( "ft_serial_number_front", ft_serial_numbers[0], std::string( FT_SERIAL_NUMBER_FRONT ) );
	nh.param<std::string>( "ft_serial_number_rear", ft_serial_numbers[1], std::string( FT_SERIAL_NUMBER_REAR ) );

	std::string lmt_yaml_file_path[2];
	nh.param<std::string>( "lmt_yaml_file_path_1", lmt_yaml_file_path[0], std::string( LMT_YAML_FILE_PATH_1 ) );
	nh.param<std::string>( "lmt_yaml_file_path_2", lmt_yaml_file_path[1], std::string( LMT_YAML_FILE_PATH_2 ) );


	// Build a model tree for the steering rate and an other for the boggie torque:
	Linear_model_tree<float> lmt_1( lmt_yaml_file_path[0], false );
	Linear_model_tree<float> lmt_2( lmt_yaml_file_path[1], false );
	int node_1 = 0;
	int node_2 = 0;


	// Identification of each FT sensor:
	int ft_ids[2] = { -1 };
	for ( int i=0 ; i < 2 ; i++ )
	{
		ros::ServiceClient svr_client = nh.serviceClient<robotiq_ft_sensor::sensor_accessor>( std::string( "ft_request_" ) + std::to_string( i ) );
		robotiq_ft_sensor::sensor_accessor srv;
		srv.request.command_id = robotiq_ft_sensor::sensor_accessor::Request::COMMAND_GET_SERIAL_NUMBER;
		if ( svr_client.call( srv ) )
		{
			ROS_INFO( "Serial number of FT sensor %i: [%s]", i, srv.response.res.c_str() );
			for ( int j=0 ; j < 2 ; j++ )
				if ( srv.response.res == ft_serial_numbers[j] )
					ft_ids[i] = j;
		}
		else
		{
			ROS_FATAL( "Failed to call service sensor_accessor" );
			return -1;
		}
	}
	if ( ft_ids[0] == -1 || ft_ids[1] == -1 )
	{
		ROS_FATAL( "Failed to identify both FT sensors" );
		return -1;
	}

	ros::Subscriber sub_w0 = nh.subscribe<geometry_msgs::WrenchStamped>( "ft_wrench_0", 1, std::bind( wrench_rcv_Callback, std::placeholders::_1, ft_ids[0] ) );
	ros::Subscriber sub_w1 = nh.subscribe<geometry_msgs::WrenchStamped>( "ft_wrench_1", 1, std::bind( wrench_rcv_Callback, std::placeholders::_1, ft_ids[1] ) );


	ros::Subscriber sub_j = nh.subscribe<rover_ctrl::Joints_info>( "joints_info", 1, joints_info_rcv_Callback );


	nav_ctrl_pub = nh.advertise<rover_ctrl::Rov_ctrl>( "nav_ctrl", 1 );
	nav_ctrl_msg.engaged = false;
	nav_ctrl_msg.speed = 0;
	nav_ctrl_msg.steer = 0;
	nav_ctrl_msg.torque = 0;
	nav_ctrl_msg.rate_mode = true;
	nav_ctrl_msg.crawling_mode = false;


	// Give the processing of the operator's inputs its own thread so that controls are passed asynchronously to the nav_node:
	ros::CallbackQueue cmd_ctrl_callback_queue;
	ros::SubscribeOptions ops_c = ros::SubscribeOptions::create<rover_ctrl::Rov_ctrl>( "cmd_ctrl", 1, cmd_ctrl_rcv_Callback, ros::VoidPtr(), &cmd_ctrl_callback_queue );
	ros::Subscriber sub_c = nh.subscribe( ops_c );
	ros::AsyncSpinner cmd_ctrl_spinner( 1, &cmd_ctrl_callback_queue );
	cmd_ctrl_spinner.start();


	int compass_fd = open_uart( COMPASS_DEVICE, COMPASS_BAUDRATE );
	float direction_angle = 0;
	float initial_direction = 0;
	bool compass_dirty = compass_get_bearing( compass_fd, initial_direction );
	float prev_direction = 0;
	float prev_measured_direction = 0;
	const float jump_threshold = 10;
	const int n_stability_checks = 3;
	int n_current_check = 0;


#ifdef USE_ONBOARD_INCLINOMETER
	int inc_fd = setup_spi( std::vector<unsigned int>{ INC_CSB } );
	usleep( 500 );
	float angle_x, angle_y;
#else
	short angle_x, angle_y;
#endif


	ROS_INFO( "Calibrating the FT sensors..." );
	std::vector<std::vector<float>> ft_means = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
	for ( int i = 0 ; i < CALIB_NB_SAMPLES ; )
	{
		usleep( (float) CALIB_DURATION/CALIB_NB_SAMPLES*1e6 );

		ros::spinOnce();

		if ( ft_dirty[0] && ft_dirty[1] )
		{
			for ( int i = 0 ; i < 4 ; i++ )
				for ( int j = 0 ; j < 3 ; j++ )
					ft_means[i][j] += ft_list[i][j]/CALIB_NB_SAMPLES;

			ft_dirty[0] = ft_dirty[1] = false;

			i++;
		}
	}
	ft_offsets = ft_means;
	std::stringstream msg;
	msg << "FT sensors calibrated with offsets:";
	for ( int i = 0 ; i < 4 ; i++ )
		for ( int j = 0 ; j < 3 ; j++ )
			msg << " " << ft_offsets[i][j];
	ROS_INFO_STREAM( msg.str() );


	float steering_rate;
	float boggie_torque;

	ros::Duration cmd_ctrl_timeout( CMD_CTRL_TIMEOUT );

	while ( ros::ok() )
	{
#ifdef USE_ONBOARD_INCLINOMETER
		get_inclinaisons( inc_fd, angle_x, angle_y );
		compass_dirty = compass_get_bearing( compass_fd, direction_angle );
#else
		compass_dirty = compass_get_all( compass_fd, direction_angle, angle_x, angle_y );
#endif

		if ( compass_dirty )
		{
			// Realign the heading direction in relation to the initial angle:
			direction_angle -= initial_direction;

			// Filter inconsistent bearing values:
			if ( fabs( direction_angle - prev_direction ) <= jump_threshold )
			{
				prev_measured_direction = direction_angle;
				prev_direction = direction_angle;
				n_current_check = 0;
			}
			else if ( fabs( direction_angle - prev_measured_direction ) > jump_threshold )
			{
				ROS_WARN( "Inconsistent jump detected in the heading direction: from %f to %f°", prev_measured_direction, direction_angle );
				prev_measured_direction = direction_angle;
				direction_angle = prev_direction;
				n_current_check = n_stability_checks;
			}
			else
			{
				prev_measured_direction = direction_angle;
				if ( --n_current_check > 0 )
					direction_angle = prev_direction;
				else
					prev_direction = direction_angle;
			}
		}


		// Retrieve latest wrenches and joint angles:
		ros::spinOnce();


		if ( ft_dirty[0] && ft_dirty[1] && joints_info_dirty )
		{
			// Flip or not the left and right to account for the robot's symmetry:
			int flip_coeff = cj_angle < 0 ? -1 : 1;

			// Build the current state:
			std::vector<float> state;
			state.push_back( flip_coeff*direction_angle );
			state.push_back( flip_coeff*cj_angle );
			state.push_back( flip_coeff*angle_x );
			state.push_back( angle_y );
			state.push_back( flip_coeff*sea_angle );
			for ( int i = 0 ; i < 4 ; i++ )
			{
				if ( i == 2 )
					continue;
				for ( int j = 0 ; j < 3 ; j++ )
					state.push_back( ( ( i + j )%2 == 0 ? 1 : flip_coeff )*ft_list[i][j] );
			}

			// Infer the controls to apply:
			if ( fabs( state[5] ) > 20 )
				steering_rate = flip_coeff*lmt_1.predict( state, node_1 );
			else
			{
				steering_rate = 0.5*state[0];
				node_1 = 0;
			}
			boggie_torque = flip_coeff*lmt_2.predict( state, node_2 );

			// Clip the control values:
			steering_rate = std::min( std::max( -STEERING_MAX_VEL, steering_rate ), STEERING_MAX_VEL );
			boggie_torque = std::min( std::max( -BOGGIE_MAX_TORQUE, boggie_torque ), BOGGIE_MAX_TORQUE );

			// Check the connection with the operator's node:
			if ( sub_c.getNumPublishers() == 0 || ros::Time::now() - last_update_cmd_ctrl >= cmd_ctrl_timeout )
				nav_ctrl_msg.engaged = false;

			// Publish the controls:
			nav_ctrl_msg.steer = steering_rate;
			nav_ctrl_msg.torque = boggie_torque;
			nav_ctrl_msg.header.stamp = ros::Time::now();
			nav_ctrl_pub.publish( nav_ctrl_msg );

			// Output the state and actions:
			printf( "%f ", ros::Time::now().toSec() );
			for ( auto val : state )
				printf( "%f ", val );
			printf( "%i %i %i %f %f\n", flip_coeff, node_1, node_2, steering_rate, boggie_torque );
			fflush( stdout );

			// Output the forces and torques:
			//printf( "%f %f %f | %f %f %f || %f %f %f | %f %f %f\n",
			        //ft_list[0][0], ft_list[0][1], ft_list[0][2], ft_list[1][0], ft_list[1][1], ft_list[1][2],
					//ft_list[2][0], ft_list[2][1], ft_list[2][2], ft_list[3][0], ft_list[3][1], ft_list[3][2] );
			//fflush( stdout );
		}


		loop_rate.sleep();
	}

	return 0;
}
