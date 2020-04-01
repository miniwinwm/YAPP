#include <p18cxxx.h>
#include <delays.h>
#include "seatalk.h"

extern volatile unsigned char seatalk_messages[SEATALK_NUMBER_OF_MESSAGES][SEATALK_MAX_MESSAGE_SIZE+1];
extern message_handler handler;
extern unsigned char autopilot_remote_command;
extern unsigned char autopilot_state;
extern volatile unsigned char seatalk_transmit_state;
extern volatile unsigned char seatalk_byte_to_write;
extern volatile unsigned char seatalk_command_bit;

void init_seatalk(message_handler callback)
{	
	handler=callback;
	
	// setup the input line
	ANSELB&=SEATALK_ANSEL_B_VAL;
	TRISB|=SEATALK_TRISB_READ_VAL;
	
	// setup the output line
	ANSELA&=SEATALK_ANSEL_A_VAL;
	TRISA&=SEATALK_TRISA_WRITE_VAL;
	SEATALK_DATA_WRITE=0;
	
	// init the soft uart timer
	PMD0bits.TMR4MD=0;	        // timer 4 module is enabled, section 3.6
	RCONbits.IPEN=0;	        // disable interrupt priority, section 9.2
	T4CON=0x08;                 // set postscalar to 2, prescalar to 1, postscalar to 1:16, timer off, section 13.1
	PR4=0xcf;                   // set timer period, section 13.1
	TMR4=0;                     // set timer initial value, section 13.1
	PIR5bits.TMR4IF=0;          // clear interrupt flag, section 9.5
	PIE5bits.TMR4IE=1;          // enable timer interrupt, section 9.6
	T4CONbits.TMR4ON=1;         // set timer on, section 13.1
	INTCONbits.GIE=1;           // globally enable interrupts, section 9.4
	INTCONbits.PEIE=1;          // enable all unmasked peripheral interrupts, section 9.4	
}

void parse_next_seatalk_message(void)
{
	static unsigned char i=0;
	
	// now look at all the messages in the messages list to see if there are any new ones ready to processing
	if(seatalk_messages[i][0]==MS_READY)
	{		
		seatalk_messages[i][0]=MS_READING;	

		if(seatalk_messages[i][1]==HEADING1)
		{
			if(seatalk_messages[i][5]==0)
			{
				autopilot_state=APS_STANDBY;
			}
			else if(seatalk_messages[i][5]==2)
			{
				autopilot_state=APS_AUTO;
			}		
			else
			{
				autopilot_state=APS_UNKNOWN;
			}

			handler(HEADING1);
		}	
		else if(seatalk_messages[i][1]==KEYSTROKE)
		{
			if(seatalk_messages[i][4]==0xfe)
			{
				autopilot_remote_command=APR_AUTO;
			}
			else if(seatalk_messages[i][4]==0xfd)
			{
				autopilot_remote_command=APR_STANDBY;
			}

			handler(KEYSTROKE);
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

unsigned char write_seatalk_sentence_with_retries(unsigned char length, unsigned char* command, unsigned char tries)
{
	unsigned char result;
	unsigned char t;

	do
	{
		result=write_seatalk_sentence(4, command);
		if(!result);	
		{
			t++;
			Delay1KTCYx(40); //delay 10ms
		}	
	} 
	while(!result && t<tries);

	return result;
}
