#include <p18cxxx.h>
#include <math.h>
#include "seatalk.h"

extern volatile unsigned char messages[NUMBER_OF_MESSAGES][MAX_MESSAGE_SIZE+1];
extern float depth;
extern float temperature;
extern float boatspeed;
extern float trip;
extern float boat_log;
extern signed char variation;
extern signed int latitude_degrees;
extern signed int longitude_degrees;
extern float latitude_minutes;
extern float longitude_minutes;
extern unsigned char hour;
extern unsigned char minute;
extern unsigned char second;
extern unsigned char year;
extern unsigned char month;
extern unsigned char date;
extern float tws;
extern float twa;
extern float sog;
extern float awa;
extern unsigned int cog;
extern float aws;
extern unsigned int heading;
extern float distance_to_destination;
extern unsigned int bearing_to_destination;
extern signed char rudder;
extern message_handler handler;

void init_seatalk(message_handler callback)
{	
	handler=callback;
	
	// setup the i/o
	TRISB|=SEATALK_TRISB_READ_VAL;
	
	// init the soft uart timer
    RCONbits.IPEN=0;
    T0CONbits.T0CS=0;
    T0CONbits.T08BIT=1;
    T0CONbits.T0PS0=0;
    T0CONbits.T0PS1=0;
    T0CONbits.T0PS2=0;
    T0CONbits.PSA=0;
    TMR0L=0x80;
    T0CONbits.TMR0ON=1;
    INTCONbits.TMR0IE=1;
    INTCONbits.GIE=1;
    INTCONbits.PEIE=1;	
}

void parse_next_seatalk_message(void)
{
	float x;
	float y;
	float z;
	float t;
	static unsigned char true_wind_data_available=0;
	static unsigned char i=0;
	unsigned char message_id;
	
	// now look at all the messages in the messages list to see if there are any new ones ready to processing
	if(messages[i][0]==MS_READY)
	{		
		messages[i][0]=MS_READING;	

		message_id=messages[i][1];

		if(message_id==APPARENT_WIND_ANGLE)
		{
			true_wind_data_available|=0x01;
			awa=((float)((((unsigned int)messages[i][3])<<8)+messages[i][4]))/2.0;
			
			handler(APPARENT_WIND_ANGLE);
		}						
		else if(message_id==APPARENT_WIND_SPEED)
		{
			true_wind_data_available|=0x02;
			aws=(float)(messages[i][3] & 0x7F) + (float)(messages[i][4]/10.0);
			handler(APPARENT_WIND_SPEED);
		}	
		else if(message_id==NAV_TO_WAYPOINT)
		{
			distance_to_destination=((float)(messages[i][6])*16.0f)+(float)(messages[i][5]>>4);

			if(messages[i][7] & 0x10)
			{
				// divide distance by 100
				distance_to_destination/=100.0f;
			}
			else
			{
				// divide distance by 10
				distance_to_destination/=10.0f;
			}

			bearing_to_destination=((unsigned int)(messages[i][4]&0x03)*90);
			bearing_to_destination+=((unsigned int)((messages[i][5]&0x0f)<<4)+(unsigned int)((messages[i][4]&0xf0)>>4))/2;
			handler(NAV_TO_WAYPOINT);
		}
		else if(message_id==SOG)
		{
			sog=(messages[i][3]+((unsigned int)(messages[i][4])<<8))/10.0;
			handler(SOG);
		}						
		else if(message_id==COG)
		{
			cog=((unsigned int)((messages[i][2]>>4)&0x03))*90;
			cog+=((unsigned int)(messages[i][3]&0x3F)*2);
			cog+=((unsigned int)(((messages[i][2]>>4)&0x0C)>>3));
			handler(COG);
		}
		else if(message_id==COMP_RUDD || message_id==COMP_RUDD_AUTO)
		{
    		unsigned char angle = (messages[i][2] >> 6) | (messages[i][3] << 2);
    		if(angle!=0xFF)    
			{
        		heading=((unsigned int)((messages[i][2]>>4)&0x03)*180)+angle;
        		if(heading>719)
				{
					heading -=720;
				}
				heading/=2;
				handler(HEADING);	
			}	

			if(message_id==COMP_RUDD)
			{
				rudder=(signed char)messages[i][4];
			}
			else
			{
				rudder=(signed char)messages[i][7];
			}
			handler(RUDDER);			
		}
		else if(message_id==GMT)												
		{
			hour=messages[i][4]; 
			minute=(messages[i][3]&0xFC)>>2;
			second=(messages[i][3]&0x03)<<4+((messages[i][2]&0xf0)>>4);
			if(second>59)
			{
				second=59;
			}
			handler(GMT);
		}	
		else if(message_id==DATE)												
		{
			year=messages[i][4]; 
			month=messages[i][2]>>4;
			date=messages[i][3];	
			handler(DATE);
		}	
		else if(message_id==DEPTH)												
		{
			depth=(float)messages[i][4];
			depth+=((float)messages[i][5])*256.0;
			depth/=32.808; 
			handler(DEPTH);
		}	
		else if(message_id==TEMPERATURE)												
		{
			temperature=(float)messages[i][3];
			temperature+=((float)messages[i][4])*256.0;
			temperature-=100.0;
			temperature/=10.0; 
			handler(TEMPERATURE);
		}	
		else if(message_id==BOATSPEED)												
		{
			true_wind_data_available|=0x04;
			boatspeed=(float)messages[i][3];
			boatspeed+=((float)messages[i][4])*256.0f;
			boatspeed/=10.0f; 
			handler(BOATSPEED);
		}	
		else if(message_id==TRIPLOG)												
		{
			boat_log=(float)messages[i][3];
			boat_log+=((float)messages[i][4])*256.0f;
			boat_log+=((float)(messages[i][2]>>4))*65536.0f;
			boat_log/=10.0f;
			
			trip=(float)messages[i][5];
			trip+=((float)messages[i][6])*256.0;
			trip+=((float)(messages[i][7]&0x0f))*65536.0;
			trip/=100.0;

			handler(TRIPLOG);
		}		
		else if(message_id==TRIP)												
		{
			trip=(float)messages[i][3];
			trip+=((float)messages[i][4])*256.0;
			trip+=((float)(messages[i][5]&0x0f))*65536.0;
			trip/=100.0;

			handler(TRIP);
		}
		else if(message_id==LOG)												
		{
			boat_log=(float)messages[i][3];
			boat_log+=((float)messages[i][4])*256.0;
			boat_log+=((float)(messages[i][5]&0x0f))*65536.0;
			boat_log/=10.0;

			handler(LOG);
		}
		else if(message_id==VARIATION)												
		{
			variation=-(signed char)(messages[i][3]);
			handler(VARIATION);
		}	
		else if(message_id==LATITUDE)												
		{
			latitude_degrees=(signed int)messages[i][3];
			latitude_minutes=messages[i][4];
			latitude_minutes+=((unsigned int)(messages[i][5]&0x7f))<<8;
			latitude_minutes/=100.0;
			if(messages[i][5]&0x80)
			{
				latitude_degrees=-latitude_degrees;
			}	
			handler(LATITUDE);
		}	
		else if(message_id==LONGITUDE)												
		{
			longitude_degrees=(signed int)messages[i][3];
			longitude_minutes=messages[i][4];
			longitude_minutes+=((unsigned int)(messages[i][5]&0x7f))<<8;
			longitude_minutes/=100.0;
			if(!(messages[i][5]&0x80))
			{
				longitude_degrees=-longitude_degrees;
			}
			handler(LONGITUDE);
		}														

		if(true_wind_data_available==0x07)
		{
			if(boatspeed<0.01f)
			{
				tws=aws;
				twa=awa;
			}
			else
			{
				x=boatspeed*sin(awa/DEGREES_TO_RADIANS);
				y=boatspeed*cos(awa/DEGREES_TO_RADIANS);
				z=aws-y;
				tws=sqrt(z*z+x*x);
				if(tws==0.0f)
				{
					twa=0.0f;
				}
				else
				{
					t=(M_PI_2-awa/DEGREES_TO_RADIANS)+acos(x/tws);
					twa=M_PI-t;
				}
				twa*=DEGREES_TO_RADIANS;
				if(twa<0.0f)
				{
					twa+=360.0f;
				}
			}
			true_wind_data_available=0;
			handler(TRUE_WIND);
		}		
				
		messages[i][0]=MS_DONE;
	}		
	
	i++;
	if(i==NUMBER_OF_MESSAGES)
	{
		i=0;
	}		
}	