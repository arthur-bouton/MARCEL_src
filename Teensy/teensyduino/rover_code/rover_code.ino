#include <SPI.h>
//#include "filters2.hh"

#define LED 13
#define BUZZ 2
#define SCK 14
#define CSB_A 9
#define CSB_B 10
#define CSB_C 15
#define SBC Serial1
#define SAB Serial3

#define ENC_ID_A 10
#define ENC_ID_B 13
#define ENC_ID_C 11

#define SPI_FREQ 100000 // Hz
#define SPI_ORDER MSBFIRST
#define SPI_MODE SPI_MODE1
#define SPI_t6t4_DELAY 10 // µs
#define SPI_t2_DELAY 40 // µs
#define SPI_t7_DELAY 50 // µs
#define SPI_t5_DELAY 1500 // µs

#define SAB_BAUD 9600
#define SAB_FORMAT SERIAL_8N1

#define SBC_BAUD 921600
#define SBC_FORMAT SERIAL_8N1

#define FRAME_LENGTH 6
#define N_LAST_FRAMES 20

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

#define CMD_SET_REF_ANGLE_A 0x51
#define CMD_SET_REF_ANGLE_B 0x52
#define CMD_SET_REF_ANGLE_C 0x54

#define HDR_CJ_ANGLE_RATE 0xCA
#define HDR_SEA_ANGLE_TOR 0x5A

#define STEERING_MAX_ANGLE 40 // °

#define BOGGIE_MAX_ANGLE 45 // °
#define SEA_STIFFNESS 0.265 // N.m/°
#define SEA_HALF_DEADZONE 1 // °

#define REF_ANGLE_A -207.1 // °
#define REF_ANGLE_B 86.3 // °
#define REF_ANGLE_C -92.1 // °

#define LOOP_PERIOD 50000 // µs
//#define LOOP_PERIOD 0 // µs

#define CMD_TIMEOUT 1000 // ms

// Motor 1 feedback control parameters:
float kp_1_pos = 2;
float ki_1_pos = 0;
#define INT_ERR_1_POS_LIMIT 100
float kp_1_vel = 1;
float ki_1_vel = 1;
#define INT_ERR_1_VEL_LIMIT 100
float max_vel_1 = 6; // °/s
float pos_precision_1 = 0.5; // °/s

// Motor 2 feedback control parameters:
float kp_2_pos = 3;
float ki_2_pos = 0;
#define INT_ERR_2_POS_LIMIT 100
float kp_2_tor = 30;
float ki_2_tor = 0;
#define INT_ERR_2_TOR_LIMIT 100
float kp_2_vel = 1;
float ki_2_vel = 1;
#define INT_ERR_2_VEL_LIMIT 100
float max_vel_2 = 30; // °/s
float pos_precision_2 = 0.5; // °/s


SPISettings pst_settings( SPI_FREQ, SPI_ORDER, SPI_MODE );

float ref_angle_A = REF_ANGLE_A;
float ref_angle_B = REF_ANGLE_B;
float ref_angle_C = REF_ANGLE_C;


elapsedMicros elapsed_loop_time;
elapsedMillis cmd_timer;

uint16_t enc_pos_A = 0;
uint16_t enc_pos_B = 0;
uint16_t enc_pos_C = 0;

float angle_A = 0;
float angle_B = 0;
float angle_C = 0;

float vel_A = 0;
float vel_B = 0;
float vel_C = 0;

elapsedMicros last_update_A = 0;
elapsedMicros last_update_B = 0;
elapsedMicros last_update_C = 0;


enum ctrl_mode { POS, VEL, TOR };

enum ctrl_mode motor_1_mode = POS;
float cmd_1_pos = 0;
float cmd_1_vel = 0;

enum ctrl_mode motor_2_mode = POS;
float cmd_2_pos = 0;
float cmd_2_tor = 0;
float cmd_2_vel = 0;

float err_1_pos = 0;
float int_err_1_pos = 0;
float err_1_vel = 0;
float int_err_1_vel = 0;

float err_2_pos = 0;
float int_err_2_pos = 0;
float err_2_tor = 0;
float int_err_2_tor = 0;
float err_2_vel = 0;
float int_err_2_vel = 0;

float pct_1 = 0;
float pct_2 = 0;

bool engaged = false;

//filters::LP_first_order<float> vel_A_filter;


class Toggle
{
	public:

	Toggle( const int pin ) : _pin( pin ), _n_switches( 0 ) {}

	bool toggle( unsigned int rep = 1, unsigned long t_high = 100, unsigned long t_low = 100, unsigned long t_end = 500, bool force = false )
	{
		if ( _n_switches == 0 || force )
		{
			_n_switches = rep;
			_t_high = t_high;
			_t_low = t_low;
			_t_end = t_end;

			_state = HIGH;
			digitalWrite( _pin, _state );
			_last_toggle = millis();
			_t_target = _t_high;
			
			return true;
		}
		return false;
	}

	void update()
	{
		if ( _n_switches != 0 && millis() - _last_toggle >= _t_target )
		{
			if ( _n_switches > 0 && _state == LOW )
				_n_switches--;
			if ( _n_switches == 0 )
				return;

			_state = ! _state;
			digitalWrite( _pin, _state );
			_last_toggle = millis();
			if ( _state == HIGH )
				_t_target = _t_high;
			else if ( _n_switches == 1 )
				_t_target = _t_end;
			else
				_t_target = _t_low;
		}
	}

	~Toggle() {}

	protected:

	int _pin;
	unsigned int _n_switches;
	unsigned long _t_high;
	unsigned long _t_low;
	unsigned long _t_end;
	unsigned long _t_target;
	unsigned long _last_toggle;
	int _state;

} led( LED ), buzz( BUZZ );


void toggle_pause( const int pin, unsigned int n = 1, unsigned long t_high = 200, unsigned long t_low = 200 )
{
	for ( int i = 0 ; i < n ; i++ )
	{
		if ( i > 0 )
			delay( t_low );

		digitalWrite( pin, HIGH );
		delay( t_high );
		digitalWrite( pin, LOW );
	}
}


uint16_t get_abs_pos( const int csb )
{
	char data_bytes[4] = { 0 };

	digitalWrite( csb, LOW );

	delayMicroseconds( SPI_t6t4_DELAY );

	SPI.transfer( 0xAA );
	delayMicroseconds( SPI_t2_DELAY );
	SPI.transfer( 0xFF );

	delayMicroseconds( SPI_t7_DELAY );

	for ( int i = 0 ; i < 4 ; i++ )
	{
		data_bytes[i] = SPI.transfer( 0xFF );
		delayMicroseconds( SPI_t2_DELAY );
	}

	for ( int i = 0 ; i < 4 ; i++ )
	{
		SPI.transfer( 0xFF );
		delayMicroseconds( SPI_t2_DELAY );
	}

	delayMicroseconds( SPI_t6t4_DELAY );

	digitalWrite( csb, HIGH );

	if ( data_bytes[0] & data_bytes[2] | data_bytes[1] & data_bytes[3] )
		return 0x8000;

	if ( ( data_bytes[1] & 3 ) == 2 )
		return 0x4000;

	return data_bytes[0] << 6 | data_bytes[1] >> 2;
}

float pos_to_degree( const int id, const uint16_t pos, const float ref_angle=0 )
{
	// PST-360 J2008813: from 1665 to 14767
	// PST-360 J2008810: from 1597 to 14714
	// PST-360 J2008811: from 1594 to 14714

	float angle;
	if ( id == 13 )
		//angle = ( pos - 1665 )*360./13102;
		angle = ( pos - 1665 )*2.7476721111280720e-2;
	else if ( id == 10 )
		//angle = ( pos - 1597 )*360./13117;
		angle = -( pos - 1597 )*2.7445299992376306e-2;
	else if ( id == 11 )
		//angle = ( pos - 1594 )*360./13120;
		angle = ( pos - 1594 )*2.7439024390243903e-2;
	else
		return 0;

	if ( ref_angle != 0 )
		angle -= ref_angle;

	angle += ( angle > 180 ? -360 : 0 ) + ( angle < -180 ? 360 : 0 );

	return angle;
}


void send_cmd_from_pct( const int id, const float pct )
{
	uint8_t msg;
	if ( id % 2 == 1 )
		msg = ( constrain( pct, -100, 100 ) + 100 )*0.63 + 1;
	else
		msg = ( constrain( pct, -100, 100 ) + 100 )*0.635 + 128.5;
	SAB.write( msg );
}

void stop_both_motors()
{
	SAB.write( 0 );
}


void setup()
{
	SAB.begin( SAB_BAUD, SAB_FORMAT );
	stop_both_motors();

	pinMode( LED, OUTPUT );
	pinMode( BUZZ, OUTPUT );

	pinMode( CSB_A, OUTPUT );
	pinMode( CSB_B, OUTPUT );
	pinMode( CSB_C, OUTPUT );
	digitalWrite( CSB_A, HIGH );
	digitalWrite( CSB_B, HIGH );
	digitalWrite( CSB_C, HIGH );
	SPI.setSCK( SCK );
	SPI.begin();

	SBC.begin( SBC_BAUD, SBC_FORMAT );

	//vel_A_filter.init_bilinear( LOOP_PERIOD*10e-6, 2*M_PI*0.1, &vel_A );

	led.toggle( 3, 200, 200 );
}


int checksum( const byte* const buffer, const int buf_pos )
{
	byte cs = 0;
	for ( int i = 0 ; i < FRAME_LENGTH ; i++ )
		cs ^= buffer[(buf_pos+i)%FRAME_LENGTH];
	return cs;
}

int read_from_sbc()
{
	byte rd_buf[FRAME_LENGTH];

	union {
		byte bytes[4];
		float fval;
	} data;
	
	if ( SBC.available() < FRAME_LENGTH )
		return 0;

	while ( SBC.available() > FRAME_LENGTH*N_LAST_FRAMES )
		SBC.read();

	for ( int i = 0 ; i < FRAME_LENGTH - 1 ; i++ )
		rd_buf[i] = SBC.read();

	int buf_pos = 0;
	while ( SBC.available() >= 1 )
	{
		rd_buf[(buf_pos+FRAME_LENGTH-1)%FRAME_LENGTH] = SBC.read();

		switch ( rd_buf[buf_pos%FRAME_LENGTH] )
		{
			case CMD_DISENGAGE :
				
				stop_both_motors();
				engaged = false;
				return 3;

			case CMD_MOTOR_1_POS :
				
				if ( checksum( rd_buf, buf_pos ) == 0 )
				{
					for ( int i = 0 ; i < FRAME_LENGTH - 2 ; i++ )
						data.bytes[i] = rd_buf[(buf_pos+1+i)%FRAME_LENGTH];

					cmd_1_pos = data.fval;

					if ( motor_1_mode != POS )
					{
						motor_1_mode = POS;
						int_err_1_pos = 0;
						int_err_1_vel = 0;
					}

					return 1;
				}
				break;

			case CMD_MOTOR_1_VEL :
				
				if ( checksum( rd_buf, buf_pos ) == 0 )
				{
					for ( int i = 0 ; i < FRAME_LENGTH - 2 ; i++ )
						data.bytes[i] = rd_buf[(buf_pos+1+i)%FRAME_LENGTH];

					cmd_1_vel = data.fval;

					if ( motor_1_mode != VEL )
					{
						motor_1_mode = VEL;
						int_err_1_pos = 0;
						int_err_1_vel = 0;
					}

					return 1;
				}
				break;

			case CMD_MOTOR_2_POS :
				
				if ( checksum( rd_buf, buf_pos ) == 0 )
				{
					for ( int i = 0 ; i < FRAME_LENGTH - 2 ; i++ )
						data.bytes[i] = rd_buf[(buf_pos+1+i)%FRAME_LENGTH];

					cmd_2_pos = data.fval;

					if ( motor_2_mode != POS )
					{
						motor_2_mode = POS;
						int_err_2_pos = 0;
						int_err_2_tor = 0;
						int_err_2_vel = 0;
					}

					return 2;
				}
				break;

			case CMD_MOTOR_2_TOR :
				
				if ( checksum( rd_buf, buf_pos ) == 0 )
				{
					for ( int i = 0 ; i < FRAME_LENGTH - 2 ; i++ )
						data.bytes[i] = rd_buf[(buf_pos+1+i)%FRAME_LENGTH];

					cmd_2_tor = data.fval;

					if ( motor_2_mode != TOR )
					{
						motor_2_mode = TOR;
						int_err_2_pos = 0;
						int_err_2_tor = 0;
						int_err_2_vel = 0;
					}

					return 2;
				}
				break;

			case CMD_ENGAGE :
				
				if ( checksum( rd_buf, buf_pos ) == 0 )
				{
					engaged = true;
					return 3;
				}
				break;

			case CMD_MOTOR_1_POS_SET_KP :
				
				if ( checksum( rd_buf, buf_pos ) == 0 )
				{
					for ( int i = 0 ; i < FRAME_LENGTH - 2 ; i++ )
						data.bytes[i] = rd_buf[(buf_pos+1+i)%FRAME_LENGTH];

					kp_1_pos = data.fval;

					return 1;
				}
				break;

			case CMD_MOTOR_1_POS_SET_KI :
				
				if ( checksum( rd_buf, buf_pos ) == 0 )
				{
					for ( int i = 0 ; i < FRAME_LENGTH - 2 ; i++ )
						data.bytes[i] = rd_buf[(buf_pos+1+i)%FRAME_LENGTH];

					ki_1_pos = data.fval;

					return 1;
				}
				break;

			case CMD_MOTOR_1_VEL_SET_KP :
				
				if ( checksum( rd_buf, buf_pos ) == 0 )
				{
					for ( int i = 0 ; i < FRAME_LENGTH - 2 ; i++ )
						data.bytes[i] = rd_buf[(buf_pos+1+i)%FRAME_LENGTH];

					kp_1_vel = data.fval;

					return 1;
				}
				break;

			case CMD_MOTOR_1_VEL_SET_KI :
				
				if ( checksum( rd_buf, buf_pos ) == 0 )
				{
					for ( int i = 0 ; i < FRAME_LENGTH - 2 ; i++ )
						data.bytes[i] = rd_buf[(buf_pos+1+i)%FRAME_LENGTH];

					ki_1_vel = data.fval;

					return 1;
				}
				break;

			case CMD_MOTOR_1_SET_MAX_VEL :
				
				if ( checksum( rd_buf, buf_pos ) == 0 )
				{
					for ( int i = 0 ; i < FRAME_LENGTH - 2 ; i++ )
						data.bytes[i] = rd_buf[(buf_pos+1+i)%FRAME_LENGTH];

					max_vel_1 = data.fval;

					return 1;
				}
				break;

			case CMD_MOTOR_1_SET_POS_PREC :
				
				if ( checksum( rd_buf, buf_pos ) == 0 )
				{
					for ( int i = 0 ; i < FRAME_LENGTH - 2 ; i++ )
						data.bytes[i] = rd_buf[(buf_pos+1+i)%FRAME_LENGTH];

					pos_precision_1 = data.fval;

					return 1;
				}
				break;

			case CMD_MOTOR_2_POS_SET_KP :
				
				if ( checksum( rd_buf, buf_pos ) == 0 )
				{
					for ( int i = 0 ; i < FRAME_LENGTH - 2 ; i++ )
						data.bytes[i] = rd_buf[(buf_pos+1+i)%FRAME_LENGTH];

					kp_2_pos = data.fval;

					return 2;
				}
				break;

			case CMD_MOTOR_2_POS_SET_KI :
				
				if ( checksum( rd_buf, buf_pos ) == 0 )
				{
					for ( int i = 0 ; i < FRAME_LENGTH - 2 ; i++ )
						data.bytes[i] = rd_buf[(buf_pos+1+i)%FRAME_LENGTH];

					ki_2_pos = data.fval;

					return 2;
				}
				break;

			case CMD_MOTOR_2_TOR_SET_KP :
				
				if ( checksum( rd_buf, buf_pos ) == 0 )
				{
					for ( int i = 0 ; i < FRAME_LENGTH - 2 ; i++ )
						data.bytes[i] = rd_buf[(buf_pos+1+i)%FRAME_LENGTH];

					kp_2_tor = data.fval;

					return 2;
				}
				break;

			case CMD_MOTOR_2_TOR_SET_KI :
				
				if ( checksum( rd_buf, buf_pos ) == 0 )
				{
					for ( int i = 0 ; i < FRAME_LENGTH - 2 ; i++ )
						data.bytes[i] = rd_buf[(buf_pos+1+i)%FRAME_LENGTH];

					ki_2_tor = data.fval;

					return 2;
				}
				break;

			case CMD_MOTOR_2_VEL_SET_KP :
				
				if ( checksum( rd_buf, buf_pos ) == 0 )
				{
					for ( int i = 0 ; i < FRAME_LENGTH - 2 ; i++ )
						data.bytes[i] = rd_buf[(buf_pos+1+i)%FRAME_LENGTH];

					kp_2_vel = data.fval;

					return 2;
				}
				break;

			case CMD_MOTOR_2_VEL_SET_KI :
				
				if ( checksum( rd_buf, buf_pos ) == 0 )
				{
					for ( int i = 0 ; i < FRAME_LENGTH - 2 ; i++ )
						data.bytes[i] = rd_buf[(buf_pos+1+i)%FRAME_LENGTH];

					ki_2_vel = data.fval;

					return 2;
				}
				break;

			case CMD_MOTOR_2_SET_MAX_VEL :
				
				if ( checksum( rd_buf, buf_pos ) == 0 )
				{
					for ( int i = 0 ; i < FRAME_LENGTH - 2 ; i++ )
						data.bytes[i] = rd_buf[(buf_pos+1+i)%FRAME_LENGTH];

					max_vel_2 = data.fval;

					return 2;
				}
				break;

			case CMD_MOTOR_2_SET_POS_PREC :
				
				if ( checksum( rd_buf, buf_pos ) == 0 )
				{
					for ( int i = 0 ; i < FRAME_LENGTH - 2 ; i++ )
						data.bytes[i] = rd_buf[(buf_pos+1+i)%FRAME_LENGTH];

					pos_precision_2 = data.fval;

					return 2;
				}
				break;

			case CMD_SET_REF_ANGLE_A :
				
				if ( checksum( rd_buf, buf_pos ) == 0 )
				{
					for ( int i = 0 ; i < FRAME_LENGTH - 2 ; i++ )
						data.bytes[i] = rd_buf[(buf_pos+1+i)%FRAME_LENGTH];

					ref_angle_A = data.fval;

					return 4;
				}
				break;

			case CMD_SET_REF_ANGLE_B :
				
				if ( checksum( rd_buf, buf_pos ) == 0 )
				{
					for ( int i = 0 ; i < FRAME_LENGTH - 2 ; i++ )
						data.bytes[i] = rd_buf[(buf_pos+1+i)%FRAME_LENGTH];

					ref_angle_B = data.fval;

					return 4;
				}
				break;

			case CMD_SET_REF_ANGLE_C :
				
				if ( checksum( rd_buf, buf_pos ) == 0 )
				{
					for ( int i = 0 ; i < FRAME_LENGTH - 2 ; i++ )
						data.bytes[i] = rd_buf[(buf_pos+1+i)%FRAME_LENGTH];

					ref_angle_C = data.fval;

					return 4;
				}
				break;
		}

		buf_pos++;
	}

	return -1;
}

bool send_to_sbc( const uint8_t hdr, const float val1=0, const float val2=0 )
{
	union raw_floats {
		float floats[2];
		uint8_t bytes[8];
	} data;
	data.floats[0] = val1;
	data.floats[1] = val2;

	uint8_t buffer[10];
	buffer[0] = hdr;
	buffer[9] = hdr;
	for ( int i = 0 ; i < 8 ; i++ )
	{
		buffer[i+1] = data.bytes[i];
		buffer[9] ^= data.bytes[i];
	}

	if ( SBC.write( buffer, 10 ) != 10 )
		return false;

	return true;
}


void compute_angle_and_vel( const int enc_id, const uint16_t enc_pos, const float ref_angle, float& angle, float& vel, elapsedMicros* last_update_ptr )
{
	if ( ( enc_pos & 0xC000 ) == 0 )
	{
		float last_angle = angle;
		angle = pos_to_degree( enc_id, enc_pos, ref_angle );
		if ( last_angle != 0 )
			vel = ( angle - last_angle )/( *last_update_ptr )*1e6;
		*last_update_ptr = 0;
	}
	else
		buzz.toggle( enc_pos == 0x8000 ? 1 : 2 );
}


void update_angles()
{
	SPI.beginTransaction( pst_settings );
	enc_pos_A = get_abs_pos( CSB_A );
	delayMicroseconds( SPI_t5_DELAY );
	enc_pos_B = get_abs_pos( CSB_B );
	delayMicroseconds( SPI_t5_DELAY );
	enc_pos_C = get_abs_pos( CSB_C );
	SPI.endTransaction();

	compute_angle_and_vel( ENC_ID_A, enc_pos_A, ref_angle_A, angle_A, vel_A, &last_update_A );
	compute_angle_and_vel( ENC_ID_B, enc_pos_B, ref_angle_B, angle_B, vel_B, &last_update_B );
	compute_angle_and_vel( ENC_ID_C, enc_pos_C, ref_angle_C, angle_C, vel_C, &last_update_C );
}


void loop()
{
	elapsed_loop_time = 0;


	update_angles();
	//vel_A_filter.update();


	//---------------------------//
	// Motor 1 feedback control: //
	//---------------------------//

	if ( motor_1_mode == POS )
	{
		err_1_pos = cmd_1_pos - angle_A;
		int_err_1_pos += err_1_pos;
		int_err_1_pos = constrain( int_err_1_pos, -INT_ERR_1_POS_LIMIT, INT_ERR_1_POS_LIMIT );
		cmd_1_vel = kp_1_pos*err_1_pos + ki_1_pos*int_err_1_pos;
		cmd_1_vel = constrain( cmd_1_vel, -max_vel_1, max_vel_1 );
	}

	err_1_vel = cmd_1_vel - vel_A;
	//err_1_vel = cmd_1_vel - vel_A_filter.get_output();
	int_err_1_vel += err_1_vel;
	int_err_1_vel = constrain( int_err_1_vel, -INT_ERR_1_VEL_LIMIT, INT_ERR_1_VEL_LIMIT );
	pct_1 = kp_1_vel*err_1_vel + ki_1_vel*int_err_1_vel;

	//if ( abs( pct_1 ) > 0.5 )
		//pct_1 += ( pct_1 > 0 ? 1 : -1 )*7;
	//else
		//pct_1 = 0;

	if ( ! engaged || abs( angle_A ) >= STEERING_MAX_ANGLE && pct_1*angle_A > 0
	               || motor_1_mode == POS && abs( cmd_1_pos - angle_A ) < pos_precision_1
				   || cmd_timer >= CMD_TIMEOUT )
	{
		pct_1 = 0;
		int_err_1_pos = 0;
		int_err_1_vel = 0;

		// For the communication of the actual desied hinge rate to the SBC:
		cmd_1_vel = 0;
	}

	//pct_1 = constrain( pct_1, -20, 20 );
	if ( engaged )
		send_cmd_from_pct( 1, pct_1 );


	//---------------------------//
	// Motor 2 feedback control: //
	//---------------------------//

	float boggie_angle = angle_C - angle_B;
	//float boggie_torque = SEA_STIFFNESS*angle_B;
	// Compute the torque applied on the boggie with a deadzone around zero:
	float boggie_torque = SEA_STIFFNESS*( angle_B > 0 ? max( angle_B - SEA_HALF_DEADZONE, 0. ) : min( angle_B + SEA_HALF_DEADZONE, 0. ) );

	if ( motor_2_mode == TOR )
	{
		err_2_tor = cmd_2_tor - boggie_torque;
		int_err_2_tor += err_2_tor;
		int_err_2_tor = constrain( int_err_2_tor, -INT_ERR_2_TOR_LIMIT, INT_ERR_2_TOR_LIMIT );
		cmd_2_vel = kp_2_tor*err_2_tor + ki_2_tor*int_err_2_tor;
	}
	else if ( motor_2_mode == POS )
	{
		cmd_2_pos = constrain( cmd_2_pos, -BOGGIE_MAX_ANGLE, BOGGIE_MAX_ANGLE );

		err_2_pos = cmd_2_pos - angle_C;
		int_err_2_pos += err_2_pos;
		int_err_2_pos = constrain( int_err_2_pos, -INT_ERR_2_POS_LIMIT, INT_ERR_2_POS_LIMIT );
		cmd_2_vel = kp_2_pos*err_2_pos + ki_2_pos*int_err_2_pos;
	}
	cmd_2_vel = constrain( cmd_2_vel, -max_vel_2, max_vel_2 );

	err_2_vel = cmd_2_vel - vel_C;
	int_err_2_vel += err_2_vel;
	int_err_2_vel = constrain( int_err_2_vel, -INT_ERR_2_VEL_LIMIT, INT_ERR_2_VEL_LIMIT );
	pct_2 = -kp_2_vel*err_2_vel - ki_2_vel*int_err_2_vel;

	if ( ! engaged || motor_2_mode == POS && abs( cmd_2_pos - angle_C ) < pos_precision_2
	               || motor_2_mode == TOR && cmd_2_tor == 0 && boggie_torque == 0
	               || cmd_timer >= CMD_TIMEOUT )
	{
		pct_2 = 0;
		int_err_2_pos = 0;
		int_err_2_tor = 0;
		int_err_2_vel = 0;
	}

	if ( engaged )
		send_cmd_from_pct( 2, pct_2 );


	// Communication with the SBC:

	//send_to_sbc( HDR_CJ_ANGLE_RATE, angle_A, vel_A );
	send_to_sbc( HDR_CJ_ANGLE_RATE, angle_A, cmd_1_vel );

	send_to_sbc( HDR_SEA_ANGLE_TOR, boggie_angle, boggie_torque );
	
	while ( read_from_sbc() > 0 ) { cmd_timer = 0; }


	led.update();
	buzz.update();


	if ( elapsed_loop_time < LOOP_PERIOD )
		delayMicroseconds( LOOP_PERIOD - elapsed_loop_time );
	else
		led.toggle();
}
