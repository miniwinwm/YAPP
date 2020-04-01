#include <p18cxxx.h>
#include <string.h>
#include <float.h>
#include <math.h>
#include <ctype.h>
#include "util.h"
#include "seatalk.h"

#ifdef _DEBUG
#include "nmea.h"	
extern char debug_string[];
#endif

extern volatile unsigned char seatalk_messages_in[SEATALK_NUMBER_OF_MESSAGES_IN][SEATALK_MAX_MESSAGE_SIZE+1];
extern seatalk_message_handler handler;
extern volatile unsigned char seatalk_transmit_state;
extern volatile unsigned char seatalk_byte_to_write;
extern volatile unsigned char seatalk_command_bit;
extern unsigned char seatalk_messages_out[SEATALK_NUMBER_OF_MESSAGES_OUT][SEATALK_MAX_MESSAGE_SIZE];
extern unsigned char seatalk_out_next_read_pos;
extern unsigned char seatalk_out_next_write_pos;
extern unsigned char seatalk_out_space;
extern volatile unsigned long millisecond_tick_count;
extern unsigned char seatalk_sentence[SEATALK_MAX_MESSAGE_SIZE];

// seatalk add more data variables here
extern float seatalk_depth;
extern float seatalk_boatspeed;
extern float seatalk_heading_magnetic;
extern float seatalk_rudder;
extern float seatalk_variation;
extern float seatalk_temperature;
extern float seatalk_trip;
extern float seatalk_log;
extern float seatalk_apparent_wind_angle;
extern float seatalk_apparent_wind_speed;
extern float seatalk_sog;
extern float seatalk_cog;
extern signed int seatalk_latitude_degrees;
extern float seatalk_latitude_minutes;
extern signed int seatalk_longitude_degrees;
extern float seatalk_longitude_minutes;
extern time_t seatalk_gmt;
extern date_t seatalk_date;

// seatalk add more data types here
static const unsigned char seatalk_identifiers[]={SEATALK_ID_DEPTH,
													SEATALK_ID_BOATSPEED,
													SEATALK_ID_COMP_RUDD,
													SEATALK_ID_VARIATION,
													SEATALK_ID_TEMPERATURE,
													SEATALK_ID_TRIPLOG,
													SEATALK_ID_TRIP,
													SEATALK_ID_LOG,
													SEATALK_ID_APPARENT_WIND_ANGLE,
													SEATALK_ID_APPARENT_WIND_SPEED,
													SEATALK_ID_SOG,							
													SEATALK_ID_COG,							
												 	SEATALK_ID_LATITUDE,						
													SEATALK_ID_LONGITUDE,					
													SEATALK_ID_GMT,							
													SEATALK_ID_DATE,	
													
													// add more message types here
												 };		

seatalk_message_type_t seatalk_identify_message_type(unsigned char message_identifier)
{
	unsigned char i;

	for(i=0; i<sizeof(seatalk_identifiers)/sizeof(seatalk_identifiers[0]);i++)
	{
		if(seatalk_identifiers[i]==message_identifier)
		{
			return i;
		}
	}

	return SEATALK_UNKNOWN;
}

unsigned char seatalk_get_identifier_from_type(seatalk_message_type_t message_type)
{
	if(message_type>=sizeof(seatalk_identifiers)/sizeof(seatalk_identifiers[0]))
	{
		return SEATALK_ID_UNKNOWN;
	}

	return seatalk_identifiers[message_type];
}

void seatalk_init(seatalk_message_handler callback)
{	
	unsigned char i;

	handler=callback;
	
	// setup the output
	seatalk_transmit_state=TS_SUCCESS;
	TRISA&=SEATALK_TRISA_WRITE_VAL;
	SEATALK_DATA_WRITE=0;
	
	// setup the input
	TRISB|=SEATALK_TRISB_READ_VAL;

	for(i=0; i<SEATALK_NUMBER_OF_MESSAGES_IN; i++)
	{
		seatalk_messages_in[i][0]=MS_DONE;
	}
}

void seatalk_send_next_message(void)
{
	const unsigned char randoms[]={23, 12, 11, 17, 24, 31, 20, 39};
	static unsigned char next_random_position=0;
	static unsigned long next_send_millisecond_time=0UL;

	if(seatalk_out_space==SEATALK_NUMBER_OF_MESSAGES_OUT)
	{
		return;
	}

	if(millisecond_tick_count<next_send_millisecond_time)
	{
		return;
	}

	if(write_seatalk_sentence(seatalk_messages_out[seatalk_out_next_read_pos][1]+3, seatalk_messages_out[seatalk_out_next_read_pos])==TRUE)
	{
		seatalk_out_space++;
		seatalk_out_next_read_pos++;
		if(seatalk_out_next_read_pos==SEATALK_NUMBER_OF_MESSAGES_OUT)
		{
			seatalk_out_next_read_pos=0;
		}
		
		next_send_millisecond_time+=10UL;
	}
	else
	{
		next_send_millisecond_time+=(unsigned long)randoms[next_random_position];
		next_random_position++;
		if(next_random_position==sizeof(randoms)/sizeof(unsigned char))
		{
			next_random_position=0;
		}
	}
}

void seatalk_process_next_message(void)
{
	static unsigned char i=0;
	unsigned char message_id;
	unsigned char c;
	
	// now look at all the messages in the messages list to see if there are any new ones ready to processing
	if(seatalk_messages_in[i][0]==MS_READY)
	{		
		seatalk_messages_in[i][0]=MS_READING;	
		message_id=seatalk_messages_in[i][1];

		if(message_id==SEATALK_ID_DEPTH)												
		{
			seatalk_depth=(float)seatalk_messages_in[i][4];
			seatalk_depth+=((float)seatalk_messages_in[i][5])*256.0f;
			seatalk_depth/=32.808f; 
			handler(SEATALK_ID_DEPTH);
		}	
		else if(message_id==SEATALK_ID_BOATSPEED)												
		{
			seatalk_boatspeed=(float)seatalk_messages_in[i][3];
			seatalk_boatspeed+=((float)seatalk_messages_in[i][4])*256.0f;
			seatalk_boatspeed/=10.0f; 
			handler(SEATALK_ID_BOATSPEED);
		}
		else if(message_id==SEATALK_ID_COMP_RUDD || message_id==SEATALK_ID_COMP_RUDD_AUTO)
		{
    		unsigned char angle = (seatalk_messages_in[i][2] >> 6) | (seatalk_messages_in[i][3] << 2);
    		if(angle!=0xFF)    
			{
        		seatalk_heading_magnetic=((unsigned int)((seatalk_messages_in[i][2]>>4)&0x03)*180)+angle;
        		if(seatalk_heading_magnetic>719)
				{
					seatalk_heading_magnetic-=720;
				}
				seatalk_heading_magnetic/=2;
				handler(SEATALK_ID_HEADING_MAGNETIC);	
			}	

			if(message_id==SEATALK_ID_COMP_RUDD)
			{
				seatalk_rudder=(signed char)seatalk_messages_in[i][4];
			}
			else
			{
				seatalk_rudder=(signed char)seatalk_messages_in[i][7];
			}
			handler(SEATALK_ID_RUDDER);			
		}
		else if(message_id==SEATALK_ID_VARIATION)												
		{
			seatalk_variation=(float)(signed char)(seatalk_messages_in[i][3]);
			handler(SEATALK_ID_VARIATION);
		}
		else if(message_id==SEATALK_ID_TEMPERATURE)												
		{
			seatalk_temperature=(float)seatalk_messages_in[i][3];
			seatalk_temperature+=((float)seatalk_messages_in[i][4])*256.0f;
			seatalk_temperature-=100.0f;
			seatalk_temperature/=10.0f; 
			handler(SEATALK_ID_TEMPERATURE);
		}
		else if(message_id==SEATALK_ID_TRIPLOG)												
		{
			seatalk_log=(float)seatalk_messages_in[i][3];
			seatalk_log+=((float)seatalk_messages_in[i][4])*256.0f;
			seatalk_log+=((float)(seatalk_messages_in[i][2]>>4))*65536.0f;
			seatalk_log/=10.0f;
			
			seatalk_trip=(float)seatalk_messages_in[i][5];
			seatalk_trip+=((float)seatalk_messages_in[i][6])*256.0f;
			seatalk_trip+=((float)(seatalk_messages_in[i][7]&0x0f))*65536.0f;
			seatalk_trip/=100.0f;

			handler(SEATALK_ID_TRIPLOG);
		}		
		else if(message_id==SEATALK_ID_TRIP)												
		{
			seatalk_trip=(float)seatalk_messages_in[i][3];
			seatalk_trip+=((float)seatalk_messages_in[i][4])*256.0f;
			seatalk_trip+=((float)(seatalk_messages_in[i][5]&0x0f))*65536.0f;
			seatalk_trip/=100.0f;

			handler(SEATALK_ID_TRIP);
		}
		else if(message_id==SEATALK_ID_LOG)												
		{
			seatalk_log=(float)seatalk_messages_in[i][3];
			seatalk_log+=((float)seatalk_messages_in[i][4])*256.0f;
			seatalk_log+=((float)(seatalk_messages_in[i][5]&0x0f))*65536.0f;
			seatalk_log/=10.0f;

			handler(SEATALK_ID_LOG);
		}
		else if(message_id==SEATALK_ID_APPARENT_WIND_ANGLE)
		{
			seatalk_apparent_wind_angle=((float)((((unsigned int)seatalk_messages_in[i][3])<<8)+seatalk_messages_in[i][4]))/2.0f;
			handler(SEATALK_ID_APPARENT_WIND_ANGLE);
		}						
		else if(message_id==SEATALK_ID_APPARENT_WIND_SPEED)
		{
			seatalk_apparent_wind_speed=(float)(seatalk_messages_in[i][3] & 0x7F) + (float)(seatalk_messages_in[i][4]/10.0f);
			handler(SEATALK_ID_APPARENT_WIND_SPEED);
		}
		else if(message_id==SEATALK_ID_SOG)
		{
			seatalk_sog=(seatalk_messages_in[i][3]+((unsigned int)(seatalk_messages_in[i][4])<<8))/10.0f;
			handler(SEATALK_ID_SOG);
		}						
		else if(message_id==SEATALK_ID_COG)
		{
			seatalk_cog=((unsigned int)((seatalk_messages_in[i][2]>>4)&0x03))*90;
			seatalk_cog+=((unsigned int)(seatalk_messages_in[i][3]&0x3F)*2);
			seatalk_cog+=((unsigned int)(((seatalk_messages_in[i][2]>>4)&0x0C)>>3));
			handler(SEATALK_ID_COG);
		}
		else if(message_id==SEATALK_ID_LATITUDE)												
		{
			seatalk_latitude_degrees=(signed int)seatalk_messages_in[i][3];
			seatalk_latitude_minutes=seatalk_messages_in[i][4];
			seatalk_latitude_minutes+=((unsigned int)(seatalk_messages_in[i][5]&0x7f))<<8;
			seatalk_latitude_minutes/=100.0f;
			if(seatalk_messages_in[i][5]&0x80)
			{
				seatalk_latitude_degrees=-seatalk_latitude_degrees;
			}	
			handler(SEATALK_ID_LATITUDE);
		}	
		else if(message_id==SEATALK_ID_LONGITUDE)												
		{
			seatalk_longitude_degrees=(signed int)seatalk_messages_in[i][3];
			seatalk_longitude_minutes=seatalk_messages_in[i][4];
			seatalk_longitude_minutes+=((unsigned int)(seatalk_messages_in[i][5]&0x7f))<<8;
			seatalk_longitude_minutes/=100.0f;
			if(!(seatalk_messages_in[i][5]&0x80))
			{
				seatalk_longitude_degrees=-seatalk_longitude_degrees;
			}
			handler(SEATALK_ID_LONGITUDE);
		}
		else if(message_id==SEATALK_ID_GMT)												
		{
			seatalk_gmt.hour=seatalk_messages_in[i][4]; 
			seatalk_gmt.minute=(seatalk_messages_in[i][3]&0xFC)>>2;
			seatalk_gmt.second=(seatalk_messages_in[i][3]&0x03)<<4+((seatalk_messages_in[i][2]&0xf0)>>4);
			if(seatalk_gmt.second>59)
			{
				seatalk_gmt.second=59;
			}
			handler(SEATALK_ID_GMT);
		}	
		else if(message_id==SEATALK_ID_DATE)												
		{
			seatalk_date.year=seatalk_messages_in[i][4]; 
			seatalk_date.month=seatalk_messages_in[i][2]>>4;
			seatalk_date.date=seatalk_messages_in[i][3];	
			handler(SEATALK_ID_DATE);
		}
		
		// seatalk add more message types here
						
		seatalk_messages_in[i][0]=MS_DONE;
	}		
	
	i++;
	if(i==SEATALK_NUMBER_OF_MESSAGES_IN)
	{
		i=0;
	}		
}	

void seatalk_queue_message_to_send(unsigned char *message)
{
	if(seatalk_out_space==0)
	{
		return;
	}

	memcpy((void *)seatalk_messages_out[seatalk_out_next_write_pos], (void *)message, (message[1]&0x0f)+3);

	seatalk_out_space--;
	seatalk_out_next_write_pos++;
	if(seatalk_out_next_write_pos==SEATALK_NUMBER_OF_MESSAGES_OUT)
	{
		seatalk_out_next_write_pos=0;
	}
}

unsigned char write_seatalk_sentence(unsigned char length, unsigned char* command)
{
	unsigned char result=FALSE;
	unsigned char i;
	
	if(seatalk_transmit_state<TS_SUCCESS)
	{
		return TS_FAILURE;
	}

	for(i=0; i<length; i++)
	{
		if(i==0)
		{
			seatalk_command_bit=1;
		}	
		else
		{
			seatalk_command_bit=0;
		}	

		seatalk_byte_to_write=command[i];
			
		seatalk_transmit_state=TS_GO;
		
		while(seatalk_transmit_state<TS_SUCCESS)
		{
		}	
		
		if(seatalk_transmit_state==TS_FAILURE)
		{
			break;
		}
	}		

	return seatalk_transmit_state==TS_SUCCESS;
}

void seatalk_depth_send(float depth)
{
	unsigned int depth_int;

	seatalk_sentence[0]=0x00;
	seatalk_sentence[1]=0x02;
	seatalk_sentence[2]=0x00;
	depth_int=(unsigned int)(depth*32.81f);
	seatalk_sentence[3]=(unsigned char)depth_int;
	seatalk_sentence[4]=(unsigned char)(depth_int>>8);
	
	seatalk_queue_message_to_send(seatalk_sentence);
}

void seatalk_boatspeed_send(float boatspeed)
{
	unsigned int boatspeed_int;

	seatalk_sentence[0]=0x20;
	seatalk_sentence[1]=0x01;
	boatspeed_int=(unsigned int)(boatspeed*10.0f);
	seatalk_sentence[2]=(unsigned char)boatspeed_int;
	seatalk_sentence[3]=(unsigned char)(boatspeed_int>>8);
	
	seatalk_queue_message_to_send(seatalk_sentence);
}

void seatalk_compass_rudder_send(float compass, float rudder)
{
	unsigned int compass_int;
	unsigned char quadrant;
	unsigned char angle;

	seatalk_sentence[0]=0x9c;
	seatalk_sentence[1]=0x01;
	seatalk_sentence[2]=0x00;
	seatalk_sentence[3]=0x00;
		
	if(compass!=FLT_MAX)
	{
		compass_int=(unsigned int)(compass*2.0f);
		quadrant=compass_int/180;
		seatalk_sentence[1]|=(quadrant<<4);
		angle=(unsigned char)(compass_int-quadrant*180);
		seatalk_sentence[1]|=(angle<<6);
		seatalk_sentence[2]=angle>>2;
	}

	if(rudder!=FLT_MAX)
	{
		seatalk_sentence[3]=(signed char)rudder;
	}

	seatalk_queue_message_to_send(seatalk_sentence);
}

void seatalk_variation_send(float variation)
{
	seatalk_sentence[0]=0x99;
	seatalk_sentence[1]=0x00;
	seatalk_sentence[2]=(char)variation;

	seatalk_queue_message_to_send(seatalk_sentence);
}

void seatalk_temperature_send(float temperature)
{
	unsigned int temperature_int=100+(unsigned int)(temperature*10.0f);
	
	seatalk_sentence[0]=0x27;
	seatalk_sentence[1]=0x01;
	seatalk_sentence[2]=(unsigned char)temperature_int;
	seatalk_sentence[3]=(unsigned char)(temperature_int>>8);

	seatalk_queue_message_to_send(seatalk_sentence);
}	

void seatalk_triplog_send(float trip, float log)
{
	unsigned long trip_int;
	unsigned long log_int;
	
	seatalk_sentence[0]=0x25;
	seatalk_sentence[1]=0x04;
	seatalk_sentence[6]=0xa0;
		
	if(log!=FLT_MAX)
	{
		log_int=(unsigned long)(log*10.0f);
		seatalk_sentence[2]=(unsigned char)log_int;
		seatalk_sentence[3]=(unsigned char)(log_int>>8);
		seatalk_sentence[1]=0x0f&((unsigned char)(log_int>>16));
		seatalk_sentence[1]<<=4;
		seatalk_sentence[1]+=0x04;
	}
	
	if(trip!=FLT_MAX)
	{
		trip_int=(unsigned long)(trip*100.0f);	
		seatalk_sentence[4]=(unsigned char)trip_int;
		seatalk_sentence[5]=(unsigned char)(trip_int>>8);
		seatalk_sentence[6]=0x0f&((unsigned char)(trip_int>>16));
		seatalk_sentence[6]+=0xa0;
	}
	
	seatalk_queue_message_to_send(seatalk_sentence);
}
	
void seatalk_trip_send(float trip)
{
	unsigned long trip_int=(unsigned long)(trip*100.0f);
	
	seatalk_sentence[0]=0x21;
	seatalk_sentence[1]=0x02;
	seatalk_sentence[2]=(unsigned char)trip_int;
	seatalk_sentence[3]=(unsigned char)(trip_int>>8);
	seatalk_sentence[4]=0x0f&((unsigned char)(trip_int>>16));
	
	seatalk_queue_message_to_send(seatalk_sentence);
}	

void seatalk_log_send(float log)
{
	unsigned long log_int=(unsigned long)(log*10.0f);

	seatalk_sentence[0]=0x22;
	seatalk_sentence[1]=0x02;
	seatalk_sentence[2]=(unsigned char)log_int;
	seatalk_sentence[3]=(unsigned char)(log_int>>8);
	seatalk_sentence[4]=0x0f&((unsigned char)(log_int>>16));
	
	seatalk_queue_message_to_send(seatalk_sentence);
}	

void seatalk_apparent_wind_angle_send(float awa)
{
	unsigned int awa_int=(unsigned int)(awa*2.0f);
	
	seatalk_sentence[0]=0x10;
	seatalk_sentence[1]=0x01;
	seatalk_sentence[2]=(unsigned char)(awa_int>>8);
	seatalk_sentence[3]=(unsigned char)awa_int;

	seatalk_queue_message_to_send(seatalk_sentence);
}	

void seatalk_apparent_wind_speed_send(float aws)
{
	seatalk_sentence[0]=0x11;
	seatalk_sentence[1]=0x01;
	seatalk_sentence[2]=0x7f&(unsigned char)aws;
	seatalk_sentence[3]=(unsigned char)(frac(aws)*10.0f);
	
	seatalk_queue_message_to_send(seatalk_sentence);
}	

void seatalk_cog_send(float cog)
{
	unsigned char quadrant=(unsigned char)(((unsigned int)cog)/90);
	unsigned int position_in_quadrant=((unsigned int)cog)%90;
	unsigned int big_remainder=position_in_quadrant>>1;
	unsigned char small_remainder=(position_in_quadrant&0x01)<<1;
	
	if(frac(cog)>=0.5f)
	{
		small_remainder++;
	}	
	seatalk_sentence[0]=0x53;
	seatalk_sentence[1]=(quadrant<<4)+(small_remainder<<6);
	seatalk_sentence[2]=big_remainder;
	seatalk_queue_message_to_send(seatalk_sentence);
}
	
void seatalk_sog_send(float sog)
{
	unsigned int sog_int=(unsigned int)(sog*10.0f);
	
	seatalk_sentence[0]=0x52;
	seatalk_sentence[1]=0x01;
	seatalk_sentence[2]=(unsigned char)sog_int;
	seatalk_sentence[3]=(unsigned char)(sog_int>>8);
	seatalk_queue_message_to_send(seatalk_sentence);
}
	
void seatalk_latitude_send(int latitude_degrees, float latitude_minutes)
{
	unsigned int latitude_minutes_int=(unsigned int)(100.0f*fabs(latitude_minutes));
	
	if(latitude_degrees<0)
	{
		latitude_minutes_int|=0x8000;
	}
		
	seatalk_sentence[0]=0x50;
	seatalk_sentence[1]=0x02;
	seatalk_sentence[2]=util_abs(latitude_degrees);
	seatalk_sentence[3]=(unsigned char)latitude_minutes_int;
	seatalk_sentence[4]=(unsigned char)(latitude_minutes_int>>8);
	seatalk_queue_message_to_send(seatalk_sentence);
}
	
void seatalk_longitude_send(int longitude_degrees, float longitude_minutes)
{
	unsigned int longitude_minutes_int=(unsigned int)(100.0f*fabs(longitude_minutes));
	
	if(longitude_minutes_int<0)
	{
		longitude_minutes_int|=0x8000;
	}
		
	seatalk_sentence[0]=0x51;
	seatalk_sentence[1]=0x02;
	seatalk_sentence[2]=util_abs(longitude_degrees);
	seatalk_sentence[3]=(unsigned char)longitude_minutes_int;
	seatalk_sentence[4]=(unsigned char)(longitude_minutes_int>>8);
	seatalk_queue_message_to_send(seatalk_sentence);
}
	
void seatalk_gmt_send(time_t time)
{
	seatalk_sentence[0]=0x54;
	seatalk_sentence[1]=1+(time.second<<4);
	seatalk_sentence[2]=(time.minute<<2)+(time.second>>4);
	seatalk_sentence[3]=time.hour;	
	seatalk_queue_message_to_send(seatalk_sentence);
}
	
void seatalk_date_send(date_t date)
{
	seatalk_sentence[0]=0x56;
	seatalk_sentence[1]=1+(date.month<<4);
	seatalk_sentence[2]=date.date;
	seatalk_sentence[3]=date.year;	
	seatalk_queue_message_to_send(seatalk_sentence);
}

// seatalk add more message types here
