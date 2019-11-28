#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <ros/ros.h>
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/String.h"
#include "robotiq_ft_sensor/sensor_accessor.h"
#include <functional>   // std::bind

#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include "gpio_access.h"


#define PRIORITY 49
#define MAX_SAFE_STACK 8*1024


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


#define LOOP_FREQ 10 // Hz


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
float fx[2] = { 0 };
float fy[2] = { 0 };
float fz[2] = { 0 };
float tx[2] = { 0 };
float ty[2] = { 0 };
float tz[2] = { 0 };

bool ft_dirty[2] = { false };

void wrench_rcv_Callback( const geometry_msgs::WrenchStamped::ConstPtr& msg, int id )
{
	ft_time[id] = msg->header.stamp.toSec();
	fx[id] = msg->wrench.force.y - offset_fy[id];
	fy[id] = msg->wrench.force.x - offset_fx[id];
	fz[id] = -msg->wrench.force.z + offset_fz[id];
	tx[id] = msg->wrench.torque.y - offset_ty[id];
	ty[id] = msg->wrench.torque.x - offset_tx[id];
	tz[id] = -msg->wrench.torque.z + offset_tz[id];

	ft_dirty[id] = true;
}


int main( int argc, char **argv )
{
	//init_rt();

	ros::init( argc, argv, "read_state" );

	ros::NodeHandle nh;

	float loop_freq;
	nh.param( "loop_rate", loop_freq, (float) LOOP_FREQ );
	ros::Rate loop_rate( loop_freq );

	std::string ft_serial_numbers[2];
	nh.param<std::string>( "ft_serial_number_front", ft_serial_numbers[0], std::string( FT_SERIAL_NUMBER_FRONT ) );
	nh.param<std::string>( "ft_serial_number_rear", ft_serial_numbers[1], std::string( FT_SERIAL_NUMBER_REAR ) );


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


	int inc_fd = setup_spi( std::vector<unsigned int>{ INC_CSB } );
	usleep( 500 );


	int16_t acc_x, acc_y;
	float angle_x, angle_y;


	ros::Time time;

	while ( ros::ok() )
	{
		acc_x = read_spi( inc_fd, INC_CSB, INC_RDAX );
		usleep( 150 );
		acc_y = read_spi( inc_fd, INC_CSB, INC_RDAY );


		angle_x = -asin( acc_y/819. )*RAD_TO_DEG - INC_OFFSET_X;
		angle_y = -asin( acc_x/819. )*RAD_TO_DEG - INC_OFFSET_Y;


		ros::spinOnce();


		if ( ft_dirty[0] && ft_dirty[1] )
		{
			time = ros::Time::now();
			printf( "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n", time.toSec(), angle_x, angle_y,
			        fx[0], fy[0], fz[0], tx[0], ty[0], tz[0], fx[1], fy[1], fz[1], tx[1], ty[1], tz[1] );
			fflush( stdout );
		}


		loop_rate.sleep();
	}

	return 0;
}
