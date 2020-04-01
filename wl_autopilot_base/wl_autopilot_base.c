#include <p18cxxx.h>
#include <delays.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "nrf24l01.h"
#include "wl_autopilot_base.h"
#include "seatalk.h"

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
#pragma config WDTEN=ON			// Watch dog timer is always enabled.
#pragma config WDTPS=1024		// Watchdog timer prescalar 
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
#pragma config CP0=OFF			// Block 0 not code-protected  
#pragma config CP1=OFF			// Block 1 not code-protected  
#pragma config CP2=OFF			// Block 2 not code-protected  
#pragma config CP3=OFF			// Block 3 not code-protected  
#pragma config CPB=OFF			// Boot block not code-protected  
#pragma config CPD=OFF			// Data EEPROM not code-protected  
#pragma config WRT0=OFF			// Block 0 not write-protected  
#pragma config WRT1=OFF			// Block 1 not write-protected  
#pragma config WRT2=OFF			// Block 2 not write-protected  
#pragma config WRT3=OFF			// Block 3 not write-protected  
#pragma config WRTC=OFF			// Configuration registers (300000-3000ffh) not write-protected  
#pragma config WRTB=OFF			// Boot Block not write-protected  
#pragma config WRTD=OFF			// Data EEPROM not write-protected  
#pragma config EBTR0=OFF		// Block 0 not protected from table reads executed in other blocks  
#pragma config EBTR1=OFF		// Block 1 not protected from table reads executed in other blocks  
#pragma config EBTR2=OFF		// Block 2 not protected from table reads executed in other blocks  
#pragma config EBTR3=OFF		// Block 3 not protected from table reads executed in other blocks  
#pragma config EBTRB=OFF		// Boot Block not protected from table reads executed in other blocks  

extern volatile unsigned char rf_read_ready;
extern unsigned char seatalk_sentence[18];
extern volatile unsigned char seatalk_byte_to_write;
extern volatile unsigned char seatalk_command_bit;
extern volatile unsigned char seatalk_transmit_state;
extern volatile unsigned char seatalk_messages[SEATALK_NUMBER_OF_MESSAGES][SEATALK_MAX_MESSAGE_SIZE+1];

static void init_app(void);
static void clear_watchdog(void);
static void write_seatalk_sentence_with_retries(unsigned char length, unsigned char* command, unsigned char tries);

void __delay_ms(unsigned int c);
void __delay_us(unsigned int c);

void low_isr(void);

void main(void) 
{ 
	unsigned char data; 
	
	// all intializations collected together here
	init_app();
	
	seatalk_sentence[0]=0x86;
	seatalk_sentence[1]=0x11;
		
	while(1)
	{
		clear_watchdog();
	
		// interrupt method
		if(rf_read_ready)
		{
			// there is data
			rf_read_ready=FALSE;
	
			// read data
			nrf24l01_read_rx_payload(&data, 1); 
			nrf24l01_irq_clear_all(); 
	
			if(data==2)
			{
				seatalk_sentence[2]=0x05;
				seatalk_sentence[3]=0xfa;
			}
			else if(data==3)
			{
				seatalk_sentence[2]=0x07;
				seatalk_sentence[3]=0xf8;
			}
			else if(data==6)
			{
				seatalk_sentence[2]=0x06;
				seatalk_sentence[3]=0xf9;				
			}
			else if(data==7)
			{
				seatalk_sentence[2]=0x08;
				seatalk_sentence[3]=0xf7;
			}			
			
			if(data==2 || data==3 || data==6 || data==7)
			{
				write_seatalk_sentence_with_retries(4, seatalk_sentence, 5);
			}
		}	
		
/* 
		// polling method
		if(nrf24l01_irq_rx_dr_active())
		{
			nrf24l01_read_rx_payload(&data, 1); //read the packet into data
			nrf24l01_irq_clear_all(); //clear all interrupts in the 24L01

			if(data==2)
			{
				seatalk_sentence[2]=0x05;
				seatalk_sentence[3]=0xfa;
			}
			else if(data==3)
			{
				seatalk_sentence[2]=0x07;
				seatalk_sentence[3]=0xf8;
			}
			else if(data==6)
			{
				seatalk_sentence[2]=0x06;
				seatalk_sentence[3]=0xf9;				
			}
			else if(data==7)
			{
				seatalk_sentence[2]=0x08;
				seatalk_sentence[3]=0xf7;
			}			
			
			if(data==2 || data==3 || data==6 || data==7)
			{
				write_seatalk_sentence_with_retries(4, seatalk_sentence, 5);
			}
		}	
*/
		// process waiting seatalk messages
		parse_next_seatalk_message();
	}	
}

static void clear_watchdog(void)
{
	ClrWdt();
}	

void __delay_us(unsigned int c)
{
	unsigned int i;

	for(i=0; i<c; i++)
	{
		Delay1TCY();
		Delay1TCY();
		Delay1TCY();
		Delay1TCY();
		Delay1TCY();
		Delay1TCY();
	}
} 	

void __delay_ms(unsigned int c)
{
	unsigned int i;
	
	for(i=0; i<c; i++) 
	{
		Delay1KTCYx(8);
	}
} 

static void write_seatalk_sentence_with_retries(unsigned char length, unsigned char* command, unsigned char tries)
{
	unsigned char i;
	const unsigned char randoms[]={23, 12, 11, 17, 44, 31, 20, 39};
	static unsigned char n=0;
	
	for(i=0; i<tries; i++)
	{
		if(write_seatalk_sentence(length, command))
		{
			break;
		}	
		
		n++;
		if(n==sizeof(randoms))
		{
			n=0;
		}	
		__delay_ms(randoms[n]);
	}	
}	

static void seatalk_message_handler(unsigned char message_type)
{		
}

static void init_app(void) 
{ 
	unsigned char i;

	OSCCONbits.IRCF=6;			// internal oscillator 4MHz
	OSCCONbits.IDLEN=0;			// SLEEP enters sleep mode
	OSCCONbits.SCS=0;			// system clock determined by config bits
	OSCTUNEbits.PLLEN=1;

	// set all pins as digital i/o
	ANSELA=0;
	ANSELB=0;
	ANSELC=0;

	__delay_ms(100);			// rf module boot time
		
	// set up rf module pins

#ifdef CW_BOARD
	TRISBbits.TRISB1=1; 		// set irq pin as input
#else
	TRISBbits.TRISB0=1; 		// set irq pin as input
#endif

	TRISCbits.TRISC2=0;			// set csn pin as output
	TRISCbits.TRISC1=0;			// set ce pin as output
	LATCbits.LATC2=1; 			// set CSN bit
	
	// init the spi port
	SSP1CON1bits.SSPEN=0;		// disable spi1
	TRISCbits.TRISC5=0;			// set mosi pin as output
	TRISCbits.TRISC4=1;      	// set miso pin as input
	TRISCbits.TRISC3=0;      	// set clk pin as output
	SSP1CON1bits.SSPM=0;
	SSP1CON1bits.CKP=0;	
	SSP1STATbits.SMP=0;	
	SSP1STATbits.CKE=1;	
	SSP1CON1bits.SSPEN=1; 		// enable spi1

	// set up rf module irq

#ifdef CW_BOARD
	INTCON2bits.INTEDG1=0;
	INTCON3bits.INT1IF=0;
#else
	INTCON2bits.INTEDG0=0;
	INTCONbits.INT0IF=0;
#endif

	// init the rf chip as transmitter
	nrf24l01_initialize_debug(true, 1, true); 

    // setup timer 6, this is used for the tick timer 
    T6CONbits.TMR6ON=0;			// disable timer 6
    T6CONbits.T6CKPS=0;         // set prescalar is 1
    T6CONbits.T6OUTPS=0;     	// set postscalar to 1
    PR6=0xd0;                   // set timer6 period
    TMR6=0;                     // set timer6
    PIR5bits.TMR6IF=0;          // clear interrupt flag
    T6CONbits.TMR6ON=1;         // enable timer 6

	// seatalk
	init_seatalk(seatalk_message_handler);

	// turn on interrupts
	RCONbits.IPEN=0;	        // disable interrupt priority, section 9.2
    PIE5bits.TMR6IE=1;          // enable timer6 interrupt 

#ifdef CW_BOARD
	INTCON3bits.INT1IE=1;		// enable rf module irq line interrupt
#else
	INTCONbits.INT0IE=1;		// enable rf module irq line interrupt
#endif

	INTCONbits.PEIE=1;          // enable all unmasked peripheral interrupts
	INTCONbits.GIE=1;           // globally enable interrupts
} 

#pragma code low_vector=0x08
void low_interrupt(void)
{
	_asm GOTO low_isr _endasm
}
	
#pragma code

#pragma interruptlow low_isr 
void low_isr(void)
{
	
	// check rf interrupt
#ifdef CW_BOARD
	if(INTCON3bits.INT1IE && INTCON3bits.INT1IF)
	{
		INTCON3bits.INT1IF=0;
		rf_read_ready=TRUE;
	}
#else
	if(INTCONbits.INT0IE && INTCONbits.INT0IF)
	{
		INTCONbits.INT0IF=0;
		rf_read_ready=TRUE;
	}
#endif

	// check tick timer
	if(PIE5bits.TMR6IE && PIR5bits.TMR6IF)
	{
		PIR5bits.TMR6IF=0;

		do_seatalk_read();
		do_seatalk_write();
	}
}
