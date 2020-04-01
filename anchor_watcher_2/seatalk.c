#include <p18cxxx.h>
#include <math.h>
#include "seatalk.h"

extern volatile unsigned char seatalk_messages[SEATALK_NUMBER_OF_MESSAGES][SEATALK_MAX_MESSAGE_SIZE+1];
extern message_handler handler;
extern float depth;
extern signed int latitude_degrees;
extern signed int longitude_degrees;
extern float latitude_minutes;
extern float longitude_minutes;
extern float sog;
extern float awa;
extern float aws;
extern unsigned int heading;

void init_seatalk(message_handler callback)
{	
	unsigned char i;

	handler=callback;
	
	// setup the input
	TRISB|=SEATALK_TRISB_READ_VAL;

	for(i=0; i<SEATALK_NUMBER_OF_MESSAGES; i++)
	{
		seatalk_messages[i][0]=MS_DONE;
	}
}

/*
void parse_next_seatalk_message(void)
{
	static unsigned char i=0;
	
	// now look at all the seatalk_messages in the seatalk_messages list to see if there are any new ones ready to processing
	if(seatalk_messages[i][0]==MS_READY)
	{		
		seatalk_messages[i][0]=MS_READING;	
		
		if(seatalk_messages[i][1]==LATITUDE)												
		{
			latitude_degrees=(signed int)seatalk_messages[i][3];
			latitude_minutes=seatalk_messages[i][4];
			latitude_minutes+=((unsigned int)(seatalk_messages[i][5]&0x7f))<<8;
			latitude_minutes/=100.0f;
			if(seatalk_messages[i][5]&0x80)
			{
				latitude_degrees=-latitude_degrees;
			}	
			handler(LATITUDE);
		}	
		else if(seatalk_messages[i][1]==LONGITUDE)												
		{
			longitude_degrees=(signed int)seatalk_messages[i][3];
			longitude_minutes=seatalk_messages[i][4];
			longitude_minutes+=((unsigned int)(seatalk_messages[i][5]&0x7f))<<8;
			longitude_minutes/=100.0;
			if(!(seatalk_messages[i][5]&0x80))
			{
				longitude_degrees=-longitude_degrees;
			}
			handler(LONGITUDE);
		}		
		else if(seatalk_messages[i][1]==CANCEL_MOB)	
		{
			if(seatalk_messages[i][3]==1)
			{
				handler(CANCEL_MOB);
			}
		}												
				
		seatalk_messages[i][0]=MS_DONE;
	}		
	
	i++;
	if(i==SEATALK_NUMBER_OF_MESSAGES)
	{
		i=0;
	}		
}	
*/

void parse_next_seatalk_message(void)
{
	static unsigned char i=0;
	
	// now look at all the seatalk_messages in the seatalk_messages list to see if there are any new ones ready to processing
	if(seatalk_messages[i][0]==MS_READY)
	{		
		seatalk_messages[i][0]=MS_READING;	

		if(seatalk_messages[i][1]==APPARENT_WIND_ANGLE)
		{
			awa=((float)((((unsigned int)seatalk_messages[i][3])<<8)+seatalk_messages[i][4]))/2.0;
			
			handler(APPARENT_WIND_ANGLE);
		}						
		else if(seatalk_messages[i][1]==APPARENT_WIND_SPEED)
		{
			aws=(float)(seatalk_messages[i][3] & 0x7F) + (float)(seatalk_messages[i][4]/10.0);
			handler(APPARENT_WIND_SPEED);
		}	
		else if(seatalk_messages[i][1]==SOG)
		{
			sog=(seatalk_messages[i][3]+((unsigned int)(seatalk_messages[i][4])<<8))/10.0;
			handler(SOG);
		}						
		else if(seatalk_messages[i][1]==HEADING1 || seatalk_messages[i][1]==HEADING2)
		{
    		unsigned char angle = (seatalk_messages[i][2] >> 6) | (seatalk_messages[i][3] << 2);
    		if(angle!=0xFF)    
			{
        		heading=((unsigned int)((seatalk_messages[i][2]>>4)&0x03)*180)+angle;
        		if(heading>719)
				{
					heading -=720;
				}
				heading/=2;
				handler(HEADING2);	
			}				
		}		
		else if(seatalk_messages[i][1]==DEPTH)												
		{
			depth=(float)seatalk_messages[i][4];
			depth+=((float)seatalk_messages[i][5])*256.0;
			depth/=32.808; 
			handler(DEPTH);
		}								
		else if(seatalk_messages[i][1]==LATITUDE)												
		{
			latitude_degrees=(signed int)seatalk_messages[i][3];
			latitude_minutes=seatalk_messages[i][4];
			latitude_minutes+=((unsigned int)(seatalk_messages[i][5]&0x7f))<<8;
			latitude_minutes/=100.0;
			if(seatalk_messages[i][5]&0x80)
			{
				latitude_degrees=-latitude_degrees;
			}	
			handler(LATITUDE);
		}	
		else if(seatalk_messages[i][1]==LONGITUDE)												
		{
			longitude_degrees=(signed int)seatalk_messages[i][3];
			longitude_minutes=seatalk_messages[i][4];
			longitude_minutes+=((unsigned int)(seatalk_messages[i][5]&0x7f))<<8;
			longitude_minutes/=100.0;
			if(!(seatalk_messages[i][5]&0x80))
			{
				longitude_degrees=-longitude_degrees;
			}
			handler(LONGITUDE);
		}														
				
		seatalk_messages[i][0]=MS_DONE;
	}		
	
	i++;
	if(i==SEATALK_NUMBER_OF_MESSAGES)
	{
		i=0;
	}		
}

