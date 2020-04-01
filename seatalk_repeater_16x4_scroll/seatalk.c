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
extern float tws;
extern float twa;
extern float awa;
extern float aws;
extern int heading;
extern int heading_true;
extern float sog;
extern unsigned int cog;
extern float temperature;
extern message_handler handler;
extern unsigned char lamps;
extern unsigned long ttg;
extern float distance_to_destination;
extern unsigned int bearing_to_destination;

void init_seatalk(message_handler callback)
{	
	handler=callback;
	
	// setup the i/o
	ANSELB&=SEATALK_ANSEL_B_VAL;
	TRISB|=SEATALK_TRISB_READ_VAL;
	
	// init the soft uart timer
	PMD0bits.TMR4MD=0;	        // timer 4 module is enabled, section 3.6
	RCONbits.IPEN=0;	        // disable interrupt priority, section 9.2
	T4CON=0x08;                 // set postscalar to 2, prescalar to 1, postscalar to 1:16, timer off, section 13.1
	PR4=0x33;                   // set timer period, section 13.1
	TMR4=0;                     // set timer initial value, section 13.1
	PIR5bits.TMR4IF=0;          // clear interrupt flag, section 9.5
	PIE5bits.TMR4IE=1;          // enable timer interrupt, section 9.6
	T4CONbits.TMR4ON=1;         // set timer on, section 13.1
	INTCONbits.GIE=1;           // globally enable interrupts, section 9.4
	INTCONbits.PEIE=1;          // enable all unmasked peripheral interrupts, section 9.4		
}

void parse_next_seatalk_message(void)
{
	float x;
	float y;
	float z;
	float t;
	static unsigned char true_wind_data_available=0;
	static unsigned char i=0;
	static unsigned char ttg_data_available=0;
	unsigned int bearing_to_destination_true;
	int off_course;
	float vmg_to_destination;
	
	// now look at all the messages in the messages list to see if there are any new ones ready to processing
	if(messages[i][0]==MS_READY)
	{		
		messages[i][0]=MS_READING;	
		if(messages[i][1]==APPARENT_WIND_ANGLE)
		{
			true_wind_data_available|=0x01;
			awa=((float)((((unsigned int)messages[i][3])<<8)+messages[i][4]))/2.0;
			
			handler(APPARENT_WIND_ANGLE);
		}						
		else if(messages[i][1]==APPARENT_WIND_SPEED)
		{
			true_wind_data_available|=0x02;
			aws=(float)(messages[i][3] & 0x7F) + (float)(messages[i][4]/10.0);
			handler(APPARENT_WIND_SPEED);
		}	
		else if(messages[i][1]==TEMPERATURE)												
		{
			temperature=(float)messages[i][3];
			temperature+=((float)messages[i][4])*256.0;
			temperature-=100.0;
			temperature/=10.0; 
			handler(TEMPERATURE);
		}
		else if(messages[i][1]==SOG)
		{
			ttg_data_available|=0x02;
			sog=(messages[i][3]+((unsigned int)(messages[i][4])<<8))/10.0;
			handler(SOG);
		}						
		else if(messages[i][1]==NAV_TO_WAYPOINT)
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

			ttg_data_available|=0x01;

			bearing_to_destination=((unsigned int)(messages[i][4]&0x03)*90);
			bearing_to_destination+=((unsigned int)((messages[i][5]&0x0f)<<4)+(unsigned int)((messages[i][4]&0xf0)>>4))/2;
			handler(NAV_TO_WAYPOINT);
		}
		else if(messages[i][1]==COG)
		{
			cog=((unsigned int)((messages[i][2]>>4)&0x03))*90;
			cog+=((unsigned int)(messages[i][3]&0x3F)*2);
			cog+=((unsigned int)(((messages[i][2]>>4)&0x0C)>>3));
			handler(COG);
		}
		else if(messages[i][1]==HEADING1 || messages[i][1]==HEADING2)
		{
			unsigned char angle=(messages[i][2] >> 6) | (messages[i][3] << 2);
			if(angle!=0xFF)    
			{
				heading=((unsigned int)((messages[i][2]>>4)&0x03)*180)+angle;
				if(heading>719)
				{
					heading -=720;
				}
				heading/=2;
				heading_true=variation+heading;
				handler(messages[i][1]); 
			}    
		}	
		else if(messages[i][1]==DEPTH)												
		{
			depth=(float)messages[i][4];
			depth+=((float)messages[i][5])*256.0;
			depth/=32.808; 
			handler(DEPTH);
		}	
		else if(messages[i][1]==BOATSPEED)												
		{
			true_wind_data_available|=0x04;
			boatspeed=(float)messages[i][3];
			boatspeed+=((float)messages[i][4])*256.0;
			boatspeed/=10.0; 
			handler(BOATSPEED);
		}	
		else if(messages[i][1]==TRIPLOG)												
		{
			boat_log=(float)messages[i][3];
			boat_log+=((float)messages[i][4])*256.0;
			boat_log+=((float)(messages[i][2]>>4))*65536.0;
			boat_log/=10.0;
			
			trip=(float)messages[i][5];
			trip+=((float)messages[i][6])*256.0;
			trip+=((float)(messages[i][7]&0x0f))*65536.0;
			trip/=100.0;

			handler(TRIPLOG);
		}				
		else if(messages[i][1]==VARIATION)												
		{
			ttg_data_available|=0x04;
			variation=-(signed char)(messages[i][3]);
			handler(VARIATION);
		}									
  		else if(messages[i][1]==LAMPS1 || messages[i][1]==LAMPS2)
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
			handler(messages[i][1]);
		}

		if(ttg_data_available==0x07)		
		{
			bearing_to_destination_true=variation+bearing_to_destination;
			off_course=bearing_to_destination_true-cog;
			if(off_course<0)
			{
				off_course=-off_course;
			}
			if(off_course>180)
			{
				off_course=360-off_course;
			}

			vmg_to_destination=cos(off_course/DEGREES_TO_RADIANS)*sog;
			if(vmg_to_destination>0.0f)
			{
				// calculate time in seconds
				ttg=(unsigned long)((distance_to_destination*3600.0f)/vmg_to_destination);
			}
			else
			{
				ttg=0UL;
			}
			handler(TIME_TO_GO);
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