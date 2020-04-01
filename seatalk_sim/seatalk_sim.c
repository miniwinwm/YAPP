#include <p18cxxx.h>
#include <delays.h>
#include <string.h>
#include <stdio.h>
#include "seatalk.h"
#include "seatalk_sim.h"

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

// for 64MHz operation the following delays can be used
// Delay1TCY(); //delays 1/16us
// Delay10TCYx(16); //delay 10us
// Delay10TCYx(160); //delay 100us
// Delay100TCYx(160); //delay 1ms
// Delay1KTCYx(160); //delay 10ms
// Delay10KTCYx(160); //delay 100ms

void flash_led(unsigned char times);

void main(void) 
{ 
	unsigned char seatalk_sentence[18];
	unsigned char result;
	unsigned char tries;

	OSCCONbits.IRCF=7;			// internal oscillator 16MHz, section 2.2.2
	OSCCONbits.IDLEN=0;			// SLEEP enters sleep mode, section 2.2.4
	OSCCONbits.SCS=0;			// system clock determined by config bits, section 2.3
	OSCTUNEbits.PLLEN=1;
		
	// all intializations collected together here
	init_app();

	// flash led twice on power up
	flash_led(10);
		
	while(1)
	{	
		// heading/rudder version 1
		do
		{
			memcpypgm2ram((char *)seatalk_sentence, "\x84\x06\x3f\x00\x00\x00\xfe\x00\x00", 9);
			result=write_seatalk_sentence(9, seatalk_sentence);
			if(!result);	
			{
				tries++;
				Delay1KTCYx(160); //delay 10ms
			}	
		} 
		while(!result && tries<100);
		Delay10KTCYx(255);

		// heading/rudder version 2
		do
		{
			memcpypgm2ram((char *)seatalk_sentence, "\x9c\x01\x3f\x02", 4);
			result=write_seatalk_sentence(4, seatalk_sentence);
			if(!result);	
			{
				tries++;
				Delay1KTCYx(160); //delay 10ms
			}	
		} 
		while(!result && tries<100);
		Delay10KTCYx(255);

		// speed through water
		do
		{
			memcpypgm2ram((char *)seatalk_sentence, "\x20\x01\x40\x00", 4);
			result=write_seatalk_sentence(4, seatalk_sentence);
			if(!result);	
			{
				tries++;
				Delay1KTCYx(160); //delay 10ms
			}	
		} 
		while(!result && tries<100);
		Delay10KTCYx(255);

		// trip log
		do
		{
			memcpypgm2ram((char *)seatalk_sentence, "\x21\x02\x10\x00\x0f", 5);
			result=write_seatalk_sentence(5, seatalk_sentence);
			if(!result);	
			{
				tries++;
				Delay1KTCYx(160); //delay 10ms
			}	
		} 
		while(!result && tries<100);
		Delay10KTCYx(255);

		// total log
		do
		{
			memcpypgm2ram((char *)seatalk_sentence, "\x22\x02\x10\x00\x0f", 5);
			result=write_seatalk_sentence(5, seatalk_sentence);
			if(!result);	
			{
				tries++;
				Delay1KTCYx(160); //delay 10ms
			}	
		} 
		while(!result && tries<100);
		Delay10KTCYx(255);

		// total and trip log
		do
		{
			memcpypgm2ram((char *)seatalk_sentence, "\x25\x04\x12\x34\x12\x34\xa0", 7);
			result=write_seatalk_sentence(7, seatalk_sentence);
			if(!result);	
			{
				tries++;
				Delay1KTCYx(160); //delay 10ms
			}	
		} 
		while(!result && tries<100);
		Delay10KTCYx(255);	

		// water temp
		do
		{
			memcpypgm2ram((char *)seatalk_sentence, "\x27\x01\xff\x00", 4);
			result=write_seatalk_sentence(4, seatalk_sentence);
			if(!result);	
			{
				tries++;
				Delay1KTCYx(160); //delay 10ms
			}	
		} 
		while(!result && tries<100);
		Delay10KTCYx(255);

		// depth
		do
		{
			memcpypgm2ram((char *)seatalk_sentence, "\x00\x02\x00\x64\x00", 5);
			result=write_seatalk_sentence(5, seatalk_sentence);
			if(!result);	
			{
				tries++;
				Delay1KTCYx(160); //delay 10ms
			}	
		} 
		while(!result && tries<100);
		Delay10KTCYx(255);
		
		// sog
		do
		{
			memcpypgm2ram((char *)seatalk_sentence, "\x52\x01\x7b\x00", 4);
			result=write_seatalk_sentence(4, seatalk_sentence);
			if(!result);	
			{
				tries++;
				Delay1KTCYx(160); //delay 10ms
			}	
		} 
		while(!result && tries<100);
		Delay10KTCYx(255);
		
		// cog
		do
		{
			memcpypgm2ram((char *)seatalk_sentence, "\x53\x00\x7b", 3);
			result=write_seatalk_sentence(3, seatalk_sentence);
			if(!result);	
			{
				tries++;
				Delay1KTCYx(160); //delay 10ms
			}	
		} 
		while(!result && tries<100);
		Delay10KTCYx(255);
		
		// lat
		do
		{
			memcpypgm2ram((char *)seatalk_sentence, "\x50\x02\x34\xb4\x14", 5);
			result=write_seatalk_sentence(5, seatalk_sentence);
			if(!result);	
			{
				tries++;
				Delay1KTCYx(160); //delay 10ms
			}	
		} 
		while(!result && tries<100);
		Delay10KTCYx(255);	
		
		// long
		do
		{
			memcpypgm2ram((char *)seatalk_sentence, "\x51\x02\x04\xcc\x09", 5);
			result=write_seatalk_sentence(5, seatalk_sentence);
			if(!result);	
			{
				tries++;
				Delay1KTCYx(160); //delay 10ms
			}	
		} 
		while(!result && tries<100);
		Delay10KTCYx(255);				

		// awa
		do
		{
			memcpypgm2ram((char *)seatalk_sentence, "\x10\x01\x00\x40", 4);
			result=write_seatalk_sentence(4, seatalk_sentence);
			if(!result);	
			{
				tries++;
				Delay1KTCYx(160); //delay 10ms
			}	
		} 
		while(!result && tries<100);
		Delay10KTCYx(255);	
		
		// aws
		do
		{
			memcpypgm2ram((char *)seatalk_sentence, "\x11\x01\x20\x03", 4);
			result=write_seatalk_sentence(4, seatalk_sentence);
			if(!result);	
			{
				tries++;
				Delay1KTCYx(160); //delay 10ms
			}	
		} 
		while(!result && tries<100);
		Delay10KTCYx(255);			

		// variation
		do
		{
			memcpypgm2ram((char *)seatalk_sentence, "\x99\x00\xfc", 3);
			result=write_seatalk_sentence(3, seatalk_sentence);
			if(!result);	
			{
				tries++;
				Delay1KTCYx(160); //delay 10ms
			}	
		} 
		while(!result && tries<100);
		Delay10KTCYx(255);	

		// date
		do
		{
			memcpypgm2ram((char *)seatalk_sentence, "\x56\xa1\x08\x0e", 4);
			result=write_seatalk_sentence(4, seatalk_sentence);
			if(!result);	
			{
				tries++;
				Delay1KTCYx(160); //delay 10ms
			}	
		} 
		while(!result && tries<100);
		Delay10KTCYx(255);		

		// time
		do
		{
			memcpypgm2ram((char *)seatalk_sentence, "\x54\x61\x10\x0a", 4);
			result=write_seatalk_sentence(4, seatalk_sentence);
			if(!result);	
			{
				tries++;
				Delay1KTCYx(160); //delay 10ms
			}	
		} 
		while(!result && tries<100);
		Delay10KTCYx(255);	

		// gps info
		do
		{
			memcpypgm2ram((char *)seatalk_sentence, "\xa5\x57\x7f\x8f\x00\x0a\x0b\x00\x00\x00", 10);
			result=write_seatalk_sentence(10, seatalk_sentence);
			if(!result);	
			{
				tries++;
				Delay1KTCYx(160); //delay 10ms
			}	
		} 
		while(!result && tries<100);
		Delay10KTCYx(255);
	
		// waypoint name
		do
		{ 
			memcpypgm2ram((char *)seatalk_sentence, "\x82\x05\x58\x00\xc5\x00\x7d\x00", 8);
			result=write_seatalk_sentence(8, seatalk_sentence);
			if(!result);	
			{
				tries++;
				Delay1KTCYx(160); //delay 10ms
			}	
		} 
		while(!result && tries<100);
		Delay10KTCYx(255);	

		// waypoint info
		do
		{
			memcpypgm2ram((char *)seatalk_sentence, "\x85\x56\x10\x42\x16\x20\x10\x00\xef", 9);
			result=write_seatalk_sentence(9, seatalk_sentence);
			if(!result);	
			{
				tries++;
				Delay1KTCYx(160); //delay 10ms
			}	
		} 
		while(!result && tries<100);
		Delay10KTCYx(255);
		
		// arrival info
		do
		{
			memcpypgm2ram((char *)seatalk_sentence, "\xa2\x44\x00\x48\x45\x4c\x50", 7);
			result=write_seatalk_sentence(7, seatalk_sentence);
			if(!result);	
			{
				tries++;
				Delay1KTCYx(160); //delay 10ms
			}	
		} 
		while(!result && tries<100);
		Delay10KTCYx(255);
	}	
}

void init_app(void)
{
	// set all pins as digital i/o
	ANSELA=0;
	ANSELB=0;
	ANSELC=0;

	// initialise seatalk
	init_seatalk();	

	// led
	TRISBbits.TRISB1=0;			// set led pin as output

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

}
