#include <p18cxxx.h>
#include <math.h>
#include "seatalk.h"

extern volatile unsigned char seatalk_messages_in[SEATALK_NUMBER_OF_MESSAGES_IN][SEATALK_MAX_MESSAGE_SIZE+1];
extern message_handler handler;
extern float seatalk_depth;
extern float seatalk_boatspeed;
extern float seatalk_heading_magnetic;
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
extern float seatalk_distance_to_destination;
extern float seatalk_bearing_to_destination;

void init_seatalk(message_handler callback)
{	
	unsigned char i;

	handler=callback;
	
	// setup the input
	TRISB|=SEATALK_TRISB_READ_VAL;

	for(i=0; i<SEATALK_NUMBER_OF_MESSAGES_IN; i++)
	{
		seatalk_messages_in[i][0]=MS_DONE;
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
		else if(message_id==SEATALK_ID_NAV_TO_WAYPOINT)
		{
			seatalk_distance_to_destination=((float)(seatalk_messages_in[i][6])*16.0f)+(float)(seatalk_messages_in[i][5]>>4);

			if(seatalk_messages_in[i][7] & 0x10)
			{
				// divide distance by 100
				seatalk_distance_to_destination/=100.0f;
			}
			else
			{
				// divide distance by 10
				seatalk_distance_to_destination/=10.0f;
			}

			seatalk_bearing_to_destination=((unsigned int)(seatalk_messages_in[i][4]&0x03)*90);
			seatalk_bearing_to_destination+=((unsigned int)((seatalk_messages_in[i][5]&0x0f)<<4)+(unsigned int)((seatalk_messages_in[i][4]&0xf0)>>4))/2;
			handler(SEATALK_ID_NAV_TO_WAYPOINT);
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
