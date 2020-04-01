#include <p18cxxx.h>
#include <stdio.h>
#include <string.h>
#include <delays.h>
#include "seatalk.h"
#include "george_corrector.h"

// Configuration bits are described in section 24.1
// Note: For a complete list of the available config pragmas and their values, in MPLAB
// click Help|Topics then select "PIC18 Config Settings" in the Language Tools section.

#pragma config FOSC=INTIO67		// Internal oscillator block, port function on RA6 and RA7 
#pragma config PLLCFG=OFF		// Oscillator PLL set in software for HFINTOSC  
#pragma config PRICLKEN=OFF		// Primary clock disabled, using INTOSC  
#pragma config FCMEN=OFF		// Fail-Safe Clock Monitor disabled  
#pragma config IESO=OFF			// Oscillator Switchover mode disabled  
#pragma config PWRTEN=OFF		// Power up timer disabled  
#pragma config BOREN=OFF		// Brown-out Reset enabled in hardware only (SBOREN is disabled) 
#pragma config BORV=285			// VBOR set to 2.85 V nominal  
#pragma config WDTEN=OFF		// Watch dog timer is always disabled. SWDTEN has no effect
#pragma config WDTPS=1			// Watchdog timer prescalar 1:1  
#pragma config CCP2MX=PORTC1	// CCP2 input/output is multiplexed with RC1  
#pragma config PBADEN=OFF		// PORTB<5:0> pins are configured as digital I/O on Reset  
#pragma config CCP3MX=PORTC6	// P3A/CCP3 input/output is mulitplexed with RC6  
#pragma config HFOFST=OFF		// HFINTOSC output and ready status are delayed by the oscillator stable status  
#pragma config T3CMX=PORTC0		// T3CKI is on RC0  
#pragma config P2BMX=PORTC0		// P2B is on RC0  
#pragma config MCLRE=EXTMCLR	// MCLR pin enabled, RE3 input pin disabled  
#pragma config STVREN=OFF		// Stack full/underflow will not cause Reset
#pragma config LVP=OFF			// Single-Supply ICSP disabled  
#pragma config XINST=OFF		// Instruction set extension and Indexed Addressing mode disabled (Legacy mode)  
#pragma config CP0=OFF			// Block 0 (000800-003fffh) not code-protected  
#pragma config CP1=OFF			// Block 1 (004000-007fffh) not code-protected  
#pragma config CPB=OFF			// Boot block (000000-0007ffh) not code-protected  
#pragma config CPD=OFF			// Data EEPROM not code-protected  
#pragma config WRT0=OFF			// Block 0 (000800-003fffh) not write-protected  
#pragma config WRT1=OFF			// Block 1 (004000-007fffh) not write-protected  
#pragma config WRTC=OFF			// Configuration registers (300000-3000ffh) not write-protected  
#pragma config WRTB=OFF			// Boot Block (000000-0007ffh) not write-protected  
#pragma config WRTD=OFF			// Data EEPROM not write-protected  
#pragma config EBTR0=OFF		// Block 0 (000800-003fffh) not protected from table reads executed in other blocks  
#pragma config EBTR1=OFF		// Block 1 (004000-007fffh) not protected from table reads executed in other blocks  
#pragma config EBTRB=OFF		// Boot Block (000000-0007ffh) not protected from table reads executed in other blocks  

extern volatile unsigned char seatalk_messages[SEATALK_NUMBER_OF_MESSAGES][SEATALK_MAX_MESSAGE_SIZE+1];
extern unsigned char autopilot_remote_command;
extern unsigned char auto_received;
extern unsigned char autopilot_required_state;
extern unsigned char autopilot_state;

void flash_led(unsigned char times);

volatile unsigned char buttons;
volatile unsigned int c; 
volatile unsigned char seatalk_byte_to_write;
volatile unsigned char seatalk_command_bit;
volatile unsigned char seatalk_transmit_state;

void main(void) 
{
	unsigned char seatalk_sentence[18];
	unsigned char result;
	unsigned char tries;

	// all intializations collected together here
	init_app();

	// flash led on power up
	flash_led(10);

	while(1)
	{	
		parse_next_seatalk_message();

		if(buttons>0)
		{
			tries=0;

			if(buttons>4)
			{
				if(buttons==5)
				{
					// -10
					do
					{
						strncpypgm2ram((char *)seatalk_sentence, "\x86\x11\x06\xf9", 4);
						result=write_seatalk_sentence(4, seatalk_sentence);
						if(!result)
						{
							tries++;
							Delay1KTCYx(160); //delay 10ms
						}	
					} 
					while(!result && tries<100);
				}
				else if(buttons==6)
				{
					// +10
					do
					{
						strncpypgm2ram((char *)seatalk_sentence, "\x86\x11\x08\xf7", 4);
						result=write_seatalk_sentence(4, seatalk_sentence);
						if(!result)
						{
							tries++;
							Delay1KTCYx(160); //delay 10ms
						}	
					}	
					while(!result && tries<100);
				}
				// long press confirmation
				if(tries==100)
				{
					flash_led(3);
				}
				else
				{
					flash_led(2);
				}
			}
			else
			{
				if(buttons==1)
				{
					// -1
					do
					{
						strncpypgm2ram((char *)seatalk_sentence, "\x86\x11\x05\xfa", 4);
						result=write_seatalk_sentence(4, seatalk_sentence);
						if(!result)
						{
							tries++;
							Delay1KTCYx(160); //delay 10ms
						}	
					}	
					while(!result && tries<100);
				}
				else if(buttons==2)
				{
					// +1
					do
					{
						strncpypgm2ram((char *)seatalk_sentence, "\x86\x11\x07\xf8", 4);
						result=write_seatalk_sentence(4, seatalk_sentence);
						if(!result)
						{
							tries++;
							Delay1KTCYx(160); //delay 10ms
						}	
					}	
					while(!result && tries<100);
				}
				// short press confirmation
				if(tries==100)
				{
					flash_led(3);
				}
				else
				{
					flash_led(1);
				}
			}

			buttons=0;
		}	
	}
}

void init_app(void)
{
	OSCCONbits.IRCF=7;			// internal oscillator 16MHz, section 2.2.2
	OSCCONbits.IDLEN=0;			// SLEEP enters sleep mode, section 2.2.4
	OSCCONbits.SCS=0;			// system clock determined by config bits, section 2.3
	OSCTUNEbits.PLLEN=1;

	c=0;
	buttons=0;

	// set all pins as digital i/o
	ANSELA=0;
	ANSELB=0;
	ANSELC=0;

	// setup the input line
	ANSELB&=SEATALK_ANSEL_B_VAL;
	TRISB|=SEATALK_TRISB_READ_VAL;
	
	// setup the output line
	ANSELA&=SEATALK_ANSEL_A_VAL;
	TRISA&=SEATALK_TRISA_WRITE_VAL;
	SEATALK_DATA_WRITE=0;

	// initialise seatalk
	init_seatalk(seatalk_message_handler);	

	// led
	TRISBbits.TRISB1=0;			// set led pin as output
	LATBbits.LATB1=1;

	// buttons
	TRISBbits.TRISB6=1;
	TRISBbits.TRISB7=1;
	WPUBbits.WPUB6=1;
	WPUBbits.WPUB7=1;
	INTCON2bits.RBPU=0;			// enable all port b pull ups
	IOCBbits.IOCB6=1;
	IOCBbits.IOCB7=1;
	INTCONbits.RBIF=0;			// clear port b change interrupt

	// button debounce and short/long press timer
	T2CONbits.TMR2ON=0;			// set timer off, section 13.1
	T2CONbits.T2CKPS=3;         // set prescalar to 16, section 13.1
	T2CONbits.T2OUTPS=0x0f;     // set postscalar to 16, section 13.1
	PR2=0xff;                   // set timer period, section 13.1
	TMR2=0;                     // set timer initial value, section 13.1
	PIR1bits.TMR2IF=0;          // clear interrupt flag, section 9.5
	PIE1bits.TMR2IE=0;          // enable timer interrupt, section 9.6
	T2CONbits.TMR2ON=1;         // set timer on, section 13.1

	// turn on interrupts
	INTCONbits.RBIE=1;			// enable port b change interrupts
	INTCONbits.PEIE=1;          // enable all unmasked peripheral interrupts
	INTCONbits.GIE=1;           // globally enable interrupts
}

void flash_led(unsigned char times)
{
	unsigned int i;
	
	for(i=0; i<times; i++)
	{
		LATBbits.LATB1=1;   			
		Delay10KTCYx(80);
		LATBbits.LATB1=0;   			
		Delay10KTCYx(80);
	}
}

void seatalk_message_handler(unsigned char message_type)
{								
	unsigned char seatalk_sentence[18];

	if(message_type==HEADING1)
	{
		if(autopilot_required_state==APS_AUTO && autopilot_state==APS_STANDBY)
		{
				strncpypgm2ram((char *)seatalk_sentence, "\x86\x11\x01\xfe", 4);
				write_seatalk_sentence_with_retries(4, seatalk_sentence, 100);
				LATBbits.LATB1=1;
		}
	}
	else if(message_type==KEYSTROKE)
	{
		if(autopilot_remote_command==APR_AUTO)
		{
			autopilot_required_state=APS_AUTO;
		}
		else if(autopilot_remote_command==APR_STANDBY)
		{
			autopilot_required_state=APS_STANDBY;
		}
	}
}		

// interrupt handling
void low_isr(void);
#pragma code low_vector=0x08
void low_interrupt(void)
{
	_asm GOTO low_isr _endasm
}
	
#pragma code
#pragma interruptlow low_isr
void low_isr(void)
{
	static unsigned int timer_counter=0;
	static unsigned char temp_buttons;

	// THIS BUTTON HANDLER IS FOR NORMALLY CLOSED BUTTONS
	if(INTCONbits.RBIE && INTCONbits.RBIF)
	{
		// if falling edge and not currently debouncing...
		if((PORTBbits.RB6 || PORTBbits.RB7) && !PIE1bits.TMR2IE)		
		{
			TMR2=0;
			PIE1bits.TMR2IE=1;				// enable timer interrupt
			INTCONbits.RBIE=0;				// disable further port b interrupts in the meantime	
			c=0;							// clear timer count
		}
					
		INTCONbits.RBIF=0;		
	}	

	// check tick timer
	if(PIE1bits.TMR2IE && PIR1bits.TMR2IF)
	{
		PIR1bits.TMR2IF=0;
		c++;								// increment count of number of times timer interrupt has fired
		if(c==24)					
		{
			if(PORTBbits.RB6)			// is input pin still high?
			{
				temp_buttons=2;
			}
			else if(PORTBbits.RB7)			// is input pin still high?
			{
				temp_buttons=1;
			}
			else
			{
				// got a bounce or very short press which is ignored
				PIE1bits.TMR2IE=0;			// kill timer interrupts and go back to sleep in main loop
				INTCONbits.RBIE=1;			// enable port b interrupts again
			}			

		}		
		else if(c>24 && c<120)
		{
			if(!PORTBbits.RB6 && !PORTBbits.RB7)
			{
				// got a short press
				buttons=temp_buttons;		// signal main loop that a button press has happened
				INTCONbits.RBIE=1;			// enable port b interrupts again
				PIE1bits.TMR2IE=0;
			}	
		}	
		/*
		else if(c>120)
		{
			// got a long press
			buttons=temp_buttons+4;			// signal main loop that a long button press has happened
			INTCONbits.RBIE=1;				// enable port b interrupts again	
			PIE1bits.TMR2IE=0;		
		}*/
		else if(c==120)
		{
			buttons=temp_buttons+4;			// signal main loop that a long button press has happened
		}	
		else if(c>244)
		{
			INTCONbits.RBIE=1;				// enable port b interrupts again	
			PIE1bits.TMR2IE=0;				
		}		
	}
	
	if(PIR5bits.TMR4IF && PIE5bits.TMR4IE)
	{
		PIR5bits.TMR4IF=0;	 
		do_seatalk_read();
		do_seatalk_write();
	}
}
