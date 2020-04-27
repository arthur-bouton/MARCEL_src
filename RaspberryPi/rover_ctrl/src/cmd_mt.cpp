#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "geometry_msgs/WrenchStamped.h"
#include "robotiq_ft_sensor/sensor_accessor.h" // from https://github.com/ros-industrial/robotiq.git
#include <functional>   // std::bind

#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include "gpio_access.h"

#include "ModelTree/cpp/model_tree.hh" // git clone https://github.com/Bouty92/ModelTree

#include "rover_ctrl/Joints_info.h"
#include "rover_ctrl/Rov_ctrl.h"


#define LMT_YAML_FILE_PATH_1 "~/Desktop/MARCEL_src/RaspberryPi/rover_ctrl/tree_params_1.yaml"
#define LMT_YAML_FILE_PATH_2 "~/Desktop/MARCEL_src/RaspberryPi/rover_ctrl/tree_params_2.yaml"


#define INC_CSB 22
#define INC_RDAX 0x10
#define INC_RDAY 0x11

#define INC_OFFSET_X 0.8 // °
#define INC_OFFSET_Y 2.0 // °


#define FT_SERIAL_NUMBER_FRONT "  F-31951"
#define FT_SERIAL_NUMBER_REAR  "  F-31952"

const float offset_fx[2] = { -25.72, 5.12 };
const float offset_fy[2] = { -86.85, -165.43 };
const float offset_fz[2] = { -49.03, -39.12 };
const float offset_tx[2] = { -1.739, -2.390 };
const float offset_ty[2] = { 0.545, 0.516 };
const float offset_tz[2] = { -0.699, -0.497 };


#define RAD_TO_DEG 57.29577951308232


#define LOOP_FREQ 2 // Hz

#define CMD_CTRL_TIMEOUT 1. // s


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
		perror( "Can't open spidev" );
		exit( -1 );
	}
	if ( ioctl( spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed ) != 0 )
	{
		perror( "ioctl couldn't set writing speed" );
		exit( -1 );
	}
	if ( ioctl( spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed ) != 0 )
	{
		perror( "ioctl couldn't set reading speed" );
		exit( -1 );
	}
	if ( ioctl( spi_fd, SPI_IOC_WR_MODE, &spi_mode ) != 0 )
	{
		perror( "ioctl couldn't set writing mode" );
		exit( -1 );
	}

	if ( ioctl( spi_fd, SPI_IOC_RD_MODE, &spi_mode ) != 0 )
	{
		perror( "ioctl couldn't set reading mode" );
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
		perror( "SPI write" );
		exit( -2 );
	}

	read( fd, byte, 2 );

	int16_t acc = ( ( ( byte[0] << 8 ) + byte[1] ) >> 5 ) - 1024;

	GPIO_SET = 1 << INC_CSB;

	return acc;
}


double ft_time[2] = { 0 };
std::vector<std::vector<float>> ft_list = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
bool ft_dirty[2] = { false };

void wrench_rcv_Callback( const geometry_msgs::WrenchStamped::ConstPtr& msg, int id )
{
	ft_time[id] = msg->header.stamp.toSec();
	ft_list[id*2][0] = msg->wrench.force.y - offset_fy[id];
	ft_list[id*2][1] = msg->wrench.force.x - offset_fx[id];
	ft_list[id*2][2] = -msg->wrench.force.z + offset_fz[id];
	ft_list[id*2+1][0] = msg->wrench.torque.y - offset_ty[id];
	ft_list[id*2+1][1] = msg->wrench.torque.x - offset_tx[id];
	ft_list[id*2+1][2] = -msg->wrench.torque.z + offset_tz[id];

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
			ROS_ERROR( "Failed to call service sensor_accessor" );
			return -1;
		}
	}
	if ( ft_ids[0] == -1 || ft_ids[1] == -1 )
	{
		fprintf( stderr, "Failed to identify both FT sensors\n" );
		return -1;
	}

	ros::Subscriber sub_f = nh.subscribe<geometry_msgs::WrenchStamped>( "ft_wrench_0", 1, std::bind( wrench_rcv_Callback, std::placeholders::_1, ft_ids[0] ) );
	ros::Subscriber sub_r = nh.subscribe<geometry_msgs::WrenchStamped>( "ft_wrench_1", 1, std::bind( wrench_rcv_Callback, std::placeholders::_1, ft_ids[1] ) );


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


	float direction_angle = 0;


	int inc_fd = setup_spi( std::vector<unsigned int>{ INC_CSB } );
	usleep( 500 );
	int16_t acc_x, acc_y;
	float angle_x, angle_y;


	ros::Duration cmd_ctrl_timeout( CMD_CTRL_TIMEOUT );

	while ( ros::ok() )
	{
		// Get the accelerations from the inclinometer:
		acc_x = read_spi( inc_fd, INC_CSB, INC_RDAX );
		usleep( 150 );
		acc_y = read_spi( inc_fd, INC_CSB, INC_RDAY );

		// Convert the accelerations into tilt angles:
		angle_x = -asin( acc_y/819. )*RAD_TO_DEG - INC_OFFSET_X;
		angle_y = -asin( acc_x/819. )*RAD_TO_DEG - INC_OFFSET_Y;


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
			float steering_rate = flip_coeff*lmt_1.predict( state, node_1 );
			float boggie_torque = flip_coeff*lmt_2.predict( state, node_2 );

			// Check the connection with the operator's node:
			if ( sub_c.getNumPublishers() == 0 || ros::Time::now() - last_update_cmd_ctrl >= cmd_ctrl_timeout )
				nav_ctrl_msg.engaged = false;

			// Publish the controls:
			nav_ctrl_msg.steer = steering_rate;
			nav_ctrl_msg.torque = boggie_torque;
			nav_ctrl_msg.header.stamp = ros::Time::now();
			nav_ctrl_pub.publish( nav_ctrl_msg );
		}


		loop_rate.sleep();
	}

	return 0;
}
