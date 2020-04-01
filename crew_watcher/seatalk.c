#include <p18cxxx.h>
#include <math.h>
#include "seatalk.h"

extern volatile unsigned char seatalk_messages[SEATALK_NUMBER_OF_MESSAGES][SEATALK_MAX_MESSAGE_SIZE+1];
extern float latitude_degrees;
extern float longitude_degrees;
extern float latitude_minutes;
extern float longitude_minutes;
extern message_handler handler;
extern volatile unsigned char seatalk_transmit_state;
extern volatile unsigned char seatalk_byte_to_write;
extern volatile unsigned char seatalk_command_bit;

void init_seatalk(message_handler callback)
{	
	unsigned char i;

	handler=callback;
	
	// setup the output
	seatalk_transmit_state=TS_SUCCESS;
	TRISA&=SEATALK_TRISA_WRITE_VAL;
	SEATALK_DATA_WRITE=0;
	
	// setup the input
	TRISB|=SEATALK_TRISB_READ_VAL;

	for(i=0; i<SEATALK_NUMBER_OF_MESSAGES; i++)
	{
		seatalk_messages[i][0]=MS_DONE;
	}
}

void parse_next_seatalk_message(void)
{
	static unsigned char i=0;
	
	// now look at all the messages in the messages list to see if there are any new ones ready to processing
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

unsigned char write_seatalk_sentence(unsigned char length, unsigned char* command)
{
	unsigned char result=FALSE;
	unsigned char i;
	
	while(seatalk_transmit_state<TS_SUCCESS)
	{
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