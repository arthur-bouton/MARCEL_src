//-------------------------------------------------------------------//
//
// To read everything that has been received on the serial port, do:
//
// while ( read_serial() != 0 ) {}
//
//-------------------------------------------------------------------//
#define FRAME_LENGTH 10
#define N_LAST_FRAMES 2
#define CMD_VEL 0xAA
#define CMD_ID 0xAD
#define SET_PI 0xA5
#define SET_MAXINT 0xA6
#define ID_F 0xAF
#define ID_B 0xAB


float Kp = 100;
float Ki = 10000;
int maxint = 2400;


int checksum( byte* buffer, int buf_pos )
{
	byte cs = 0;
	for ( int i = 0 ; i < FRAME_LENGTH ; i++ )
		cs ^= buffer[(buf_pos+i)%FRAME_LENGTH];
	return cs;
}

int read_serial()
{
	byte rd_buf[FRAME_LENGTH];
	union {
		byte bytes[8];
		float floats[2];
	} data;
	
	if ( Serial.available() < FRAME_LENGTH )
		return 0;

	while ( Serial.available() > FRAME_LENGTH*N_LAST_FRAMES )
		Serial.read();

	for ( int i = 0 ; i < FRAME_LENGTH - 1 ; i++ )
		rd_buf[i] = Serial.read();

	int buf_pos = 0;
	while ( Serial.available() >= 1 )
	{
		rd_buf[(buf_pos+FRAME_LENGTH-1)%FRAME_LENGTH] = Serial.read();

		switch ( rd_buf[buf_pos%FRAME_LENGTH] )
		{
			case CMD_VEL :
				
				if ( checksum( rd_buf, buf_pos ) == 0 )
				{
					for ( int i = 0 ; i < FRAME_LENGTH - 2 ; i++ )
						data.bytes[i] = rd_buf[(buf_pos+1+i)%FRAME_LENGTH];

					L_Target_recv = data.floats[0];
					R_Target_recv = data.floats[1];

					commandTimer = millis();

					return 1;
				}
				break;

			case CMD_ID :
				
				#ifdef FRONT
				Serial.write( ID_F );
				#elif BACK
				Serial.write( ID_B );
				#endif

				return 2;

			case SET_PI :
				
				if ( checksum( rd_buf, buf_pos ) == 0 )
				{
					for ( int i = 0 ; i < FRAME_LENGTH - 2 ; i++ )
						data.bytes[i] = rd_buf[(buf_pos+1+i)%FRAME_LENGTH];

					Kp = data.floats[0];
					Ki = data.floats[1];

					Serial.print( "Kp set to " ); 
					Serial.print( Kp );
					Serial.print( " and Ki set to " ); 
					Serial.println( Ki );

					return 3;
				}
				break;

			case SET_MAXINT :
				
				if ( checksum( rd_buf, buf_pos ) == 0 )
				{
					for ( int i = 0 ; i < FRAME_LENGTH - 2 ; i++ )
						data.bytes[i] = rd_buf[(buf_pos+1+i)%FRAME_LENGTH];

					if ( data.floats[0] = data.floats[1] )
					{
						maxint = data.floats[0];

						Serial.print( "maxint set to " ); 
						Serial.println( maxint );

						return 4;
					}
				}
				break;
		}

		buf_pos++;
	}

	return -1;
}
