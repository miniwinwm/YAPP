#include <p18cxxx.h>
#include <math.h>
#include <delays.h>
#include "seatalk.h"

// for 64MHz operation the following delays can be used
// Delay1TCY(); //delays 1/16us
// Delay10TCYx(16); //delay 10us
// Delay10TCYx(160); //delay 100us
// Delay100TCYx(160); //delay 1ms
// Delay1KTCYx(160); //delay 10ms
// Delay10KTCYx(160); //delay 100ms

void init_seatalk()
{	
	// setup the input line
	ANSELB&=SEATALK_ANSEL_B_VAL;
	TRISB|=SEATALK_TRISB_READ_VAL;
	
	// setup the output line
	ANSELA&=SEATALK_ANSEL_A_VAL;
	TRISA&=SEATALK_TRISA_WRITE_VAL;
	SEATALK_DATA_WRITE=0;
}

unsigned char write_seatalk_byte(unsigned char byte, unsigned char command)
{
	int i;
	unsigned char b;
	
	if(command)
	{
		unsigned char counter=0;
		while(counter<20)
		{
			if(SEATALK_DATA_READ==0)
			{
				return FALSE;
			}	
			counter++;
			Delay100TCYx(160); //delay 1ms
		}	
	}	
	
	// start bit
	SEATALK_DATA_WRITE=1;
	Delay10TCYx(160); //delay 100us
	Delay10TCYx(166); //delay 108us
	if(SEATALK_DATA_READ==1)
	{
		SEATALK_DATA_WRITE=0;
		return FALSE;
	}
	
	// data
	for(i=0; i<8; i++)
	{
		b=byte&0x01;
		byte>>=1;
		SEATALK_DATA_WRITE=!b;
		Delay10TCYx(160); //delay 100us
		Delay10TCYx(166); //delay 108us
		if(SEATALK_DATA_READ!=b)
		{
			SEATALK_DATA_WRITE=0;
			return FALSE;
		}
	}	
	
	// command bit
	SEATALK_DATA_WRITE=!command;
	Delay10TCYx(160); //delay 100us
	Delay10TCYx(166); //delay 108us
	if(SEATALK_DATA_READ!=command)
	{
		SEATALK_DATA_WRITE=0;
		return FALSE;
	}
		
	// stop bit
	SEATALK_DATA_WRITE=0;
	Delay10TCYx(160); //delay 100us
	Delay10TCYx(166); //delay 108us
	if(SEATALK_DATA_READ==0)
	{
		return FALSE;
	}	
	return TRUE;
}	

unsigned char write_seatalk_sentence(unsigned char length, unsigned char* command)
{
	unsigned char result=FALSE;
	unsigned char i;
	
	for(i=0; i<length; i++)
	{
		result=write_seatalk_byte(command[i], i==0);
		if(!result)
		{
			break;
		}
	}		

	return result;
}	
