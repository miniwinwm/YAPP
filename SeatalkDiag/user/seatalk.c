#include <p18cxxx.h>
#include <math.h>
#include "seatalk.h"

extern unsigned int flags;
extern float xte;
extern char direction_to_steer;
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
extern message_handler handler;
extern unsigned char lamps;
extern unsigned char keystroke;
extern unsigned char autopilot_state;
extern unsigned char arrival_circle_entered;
extern unsigned char perpendicular_passed;
extern char waypoint_id[4];

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
	static unsigned char i=0;
	
	// now look at all the messages in the messages list to see if there are any new ones ready to processing
	if(messages[i][0]==MS_READY)
	{		
		messages[i][0]=MS_READING;	

		if(messages[i][1]==APPARENT_WIND_ANGLE)
		{
			awa=((float)((((unsigned int)messages[i][3])<<8)+messages[i][4]))/2.0;
			
			handler(APPARENT_WIND_ANGLE, &(messages[i][1]));
		}						
		else if(messages[i][1]==APPARENT_WIND_SPEED)
		{
			aws=(float)(messages[i][3] & 0x7F) + (float)(messages[i][4]/10.0);
			handler(APPARENT_WIND_SPEED, &(messages[i][1]));
		}	
		else if (messages[i][1]==WAYPOINT_ID)
		{
			if ((messages[i][3] + messages[i][4] == 0xff) && (messages[i][5] + messages[i][6] == 0xff) && 
					(messages[i][7] + messages[i][8] == 0xff))
			{
				waypoint_id[0] = messages[i][3] & 0x3f;
				waypoint_id[1] = (messages[i][5]&0x0f)<<2 | (messages[i][3]&0xc0)>>6;
				waypoint_id[2] = (messages[i][7]&0x03)<<4 | (messages[i][5]&0xf0)>>4;
				waypoint_id[3] = (messages[i][7]&0xfc)>>2; 
				
				waypoint_id[0]+=0x30;
				waypoint_id[1]+=0x30;
				waypoint_id[2]+=0x30;
				waypoint_id[3]+=0x30;

				handler(WAYPOINT_ID, &(messages[i][1]));
			}	
		}	
		else if(messages[i][1]==LAMPS)
		{
			switch(messages[i][3])
			{
				case 0:
					lamps=0;
					break;
					
				case 4:
					lamps=1;
					break;
					
				case 8:
					lamps=2;
					break;
					
				case 12:
					lamps=3;
					break;
			}	
			handler(LAMPS, &(messages[i][1]));
		}
		else if(messages[i][1]==NAV_TO_WAYPOINT)
		{
			xte=((float)(messages[i][3])*16.0f)+(float)(messages[i][2]>>4);
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
	
			if (messages[i][7]&0x40)
			{
				direction_to_steer='R';
			}
			else
			{
				direction_to_steer='L';
			}			
			
			flags=messages[i][7]&0x0f;
			handler(NAV_TO_WAYPOINT, &(messages[i][1]));
		}
		else if(messages[i][1]==SOG)
		{
			sog=(messages[i][3]+((unsigned int)(messages[i][4])<<8))/10.0;
			handler(SOG, &(messages[i][1]));
		}						
		else if(messages[i][1]==COG)
		{
			cog=((unsigned int)((messages[i][2]>>4)&0x03))*90;
			cog+=((unsigned int)(messages[i][3]&0x3F)*2);
			cog+=((unsigned int)(((messages[i][2]>>4)&0x0C)>>3));
			handler(COG, &(messages[i][1]));
		}
		else if(messages[i][1]==HEADING1)
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
			}	

			if(messages[i][5]==0)
			{
				autopilot_state=APS_STANDBY;
			}
			else if(messages[i][5]==2)
			{
				autopilot_state=APS_AUTO;
			}		
			else
			{
				autopilot_state=APS_UNKNOWN;
			}

			handler(HEADING1, &(messages[i][1]));	
		}	
		else if(messages[i][1]==GMT)												
		{
			hour=messages[i][4]; 
			minute=(messages[i][3]&0xFC)>>2;
			second=(messages[i][3]&0x03)<<4+((messages[i][2]&0xf0)>>4);
			handler(GMT, &(messages[i][1]));
		}	
		else if(messages[i][1]==DATE)												
		{
			year=messages[i][4]; 
			month=messages[i][2]>>4;
			date=messages[i][3];	
			handler(DATE, &(messages[i][1]));
		}	
		else if(messages[i][1]==DEPTH)												
		{
			depth=(float)messages[i][4];
			depth+=((float)messages[i][5])*256.0;
			depth/=32.808; 
			handler(DEPTH, &(messages[i][1]));
		}	
		else if(messages[i][1]==TEMPERATURE)												
		{
			temperature=(float)messages[i][3];
			temperature+=((float)messages[i][4])*256.0;
			temperature-=100.0;
			temperature/=10.0; 
			handler(TEMPERATURE, &(messages[i][1]));
		}	
		else if(messages[i][1]==BOATSPEED)												
		{
			boatspeed=(float)messages[i][3];
			boatspeed+=((float)messages[i][4])*256.0f;
			boatspeed/=10.0f; 
			handler(BOATSPEED, &(messages[i][1]));
		}	
		else if(messages[i][1]==TRIPLOG)												
		{
			boat_log=(float)messages[i][3];
			boat_log+=((float)messages[i][4])*256.0f;
			boat_log+=((float)(messages[i][2]>>4))*65536.0f;
			boat_log/=10.0f;
			
			trip=(float)messages[i][5];
			trip+=((float)messages[i][6])*256.0;
			trip+=((float)(messages[i][7]&0x0f))*65536.0;
			trip/=100.0;

			handler(TRIPLOG, &(messages[i][1]));
		}				
		else if(messages[i][1]==VARIATION)												
		{
			variation=-(signed char)(messages[i][3]);
			handler(VARIATION, &(messages[i][1]));
		}	
		else if(messages[i][1]==LATITUDE)												
		{
			latitude_degrees=(signed int)messages[i][3];
			latitude_minutes=messages[i][4];
			latitude_minutes+=((unsigned int)(messages[i][5]&0x7f))<<8;
			latitude_minutes/=100.0;
			if(messages[i][5]&0x80)
			{
				latitude_degrees=-latitude_degrees;
			}	
			handler(LATITUDE, &(messages[i][1]));
		}	
		else if(messages[i][1]==LONGITUDE)												
		{
			longitude_degrees=(signed int)messages[i][3];
			longitude_minutes=messages[i][4];
			longitude_minutes+=((unsigned int)(messages[i][5]&0x7f))<<8;
			longitude_minutes/=100.0;
			if(!(messages[i][5]&0x80))
			{
				longitude_degrees=-longitude_degrees;
			}
			handler(LONGITUDE, &(messages[i][1]));
		}		
		else if (messages[i][1]==ARRIVAL_INFO)
		{
			if (messages[i][2] & 0x40 == 0x40)
			{
				arrival_circle_entered = 1;
			}
			else
			{
				arrival_circle_entered = 0;
			}
			if (messages[i][2] & 0x20 == 0x20)
			{
				perpendicular_passed = 1;
			}
			else
			{
				perpendicular_passed = 0;
			}
			waypoint_id[0] = messages[i][4];
			waypoint_id[1] = messages[i][5];
			waypoint_id[2] = messages[i][6];
			waypoint_id[3] = messages[i][7];

			handler(ARRIVAL_INFO, &(messages[i][1]));
		}
		else if(messages[i][1]==KEYSTROKE)												
		{
			if(messages[i][4]==0xfa)	
			{
				keystroke=KS_MINUS_1;
			}	
			else if(messages[i][4]==0xf8)	
			{
				keystroke=KS_PLUS_1;
			}
			else if(messages[i][4]==0xf9)	
			{
				keystroke=KS_MINUS_10;
			}
			else if(messages[i][4]==0xf7)	
			{
				keystroke=KS_PLUS_10;
			}
			else if(messages[i][4]==0xfe)	
			{
				keystroke=KS_AUTO;
			}
			else if(messages[i][4]==0xfd)	
			{
				keystroke=KS_STANDBY;
			}
			handler(KEYSTROKE, &(messages[i][1]));
		}												
				
		messages[i][0]=MS_DONE;
	}		
	
	i++;
	if(i==NUMBER_OF_MESSAGES)
	{
		i=0;
	}		
}	