#include <p18cxxx.h>
#include "seatalk.h"
#include "seatalk_repeater_16x4.h"
#include "hd44780_16x4_lcd.h"

// Configuration bits are described in section 24.1
// Note: For a complete list of the available config pragmas and their values, in MPLAB
// click Help|Topics then select "PIC18 Config Settings" in the Language Tools section.

#ifdef __18F25K22_H
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
#pragma config CP2=OFF			// Block 2 (008000-00bfffh) not code-protected  
#pragma config CP3=OFF			// Block 3 (00c000-00ffffh) not code-protected  
#pragma config CPB=OFF			// Boot block (000000-0007ffh) not code-protected  
#pragma config CPD=OFF			// Data EEPROM not code-protected  
#pragma config WRT0=OFF			// Block 0 (000800-003fffh) not write-protected  
#pragma config WRT1=OFF			// Block 1 (004000-007fffh) not write-protected  
#pragma config WRT2=OFF			// Block 2 (008000-00bfffh) not write-protected  
#pragma config WRT3=OFF			// Block 3 (00c000-00ffffh) not write-protected  
#pragma config WRTC=OFF			// Configuration registers (300000-3000ffh) not write-protected  
#pragma config WRTB=OFF			// Boot Block (000000-0007ffh) not write-protected  
#pragma config WRTD=OFF			// Data EEPROM not write-protected  
#pragma config EBTR0=OFF		// Block 0 (000800-003fffh) not protected from table reads executed in other blocks  
#pragma config EBTR1=OFF		// Block 1 (004000-007fffh) not protected from table reads executed in other blocks  
#pragma config EBTR2=OFF		// Block 2 (008000-00bfffh) not protected from table reads executed in other blocks  
#pragma config EBTR3=OFF		// Block 3 (00c000-00ffffh) not protected from table reads executed in other blocks  
#pragma config EBTRB=OFF		// Boot Block (000000-0007ffh) not protected from table reads executed in other blocks  
#else 
#ifdef __18F26K22_H
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
#pragma config CP2=OFF			// Block 2 (008000-00bfffh) not code-protected  
#pragma config CP3=OFF			// Block 3 (00c000-00ffffh) not code-protected  
#pragma config CPB=OFF			// Boot block (000000-0007ffh) not code-protected  
#pragma config CPD=OFF			// Data EEPROM not code-protected  
#pragma config WRT0=OFF			// Block 0 (000800-003fffh) not write-protected  
#pragma config WRT1=OFF			// Block 1 (004000-007fffh) not write-protected  
#pragma config WRT2=OFF			// Block 2 (008000-00bfffh) not write-protected  
#pragma config WRT3=OFF			// Block 3 (00c000-00ffffh) not write-protected  
#pragma config WRTC=OFF			// Configuration registers (300000-3000ffh) not write-protected  
#pragma config WRTB=OFF			// Boot Block (000000-0007ffh) not write-protected  
#pragma config WRTD=OFF			// Data EEPROM not write-protected  
#pragma config EBTR0=OFF		// Block 0 (000800-003fffh) not protected from table reads executed in other blocks  
#pragma config EBTR1=OFF		// Block 1 (004000-007fffh) not protected from table reads executed in other blocks  
#pragma config EBTR2=OFF		// Block 2 (008000-00bfffh) not protected from table reads executed in other blocks  
#pragma config EBTR3=OFF		// Block 3 (00c000-00ffffh) not protected from table reads executed in other blocks  
#pragma config EBTRB=OFF		// Boot Block (000000-0007ffh) not protected from table reads executed in other blocks  
#else
#error "Processor not supported
#endif
#endif

extern volatile unsigned char messages[NUMBER_OF_MESSAGES][MAX_MESSAGE_SIZE+1];
extern volatile unsigned long tick_count;
extern volatile unsigned char tick;
extern unsigned char lamps;

void main(void) 
{ 		
	// all intializations collected together here
	init_app();

	while(1)
	{		
		parse_next_seatalk_message();
		if(tick)
		{
			update_display();
			tick=FALSE;
		}
	}	
}

void init_app(void)
{
	unsigned int j;

	OSCCONbits.IRCF=5;			// internal oscillator 4MHz, section 2.2.2
	OSCCONbits.IDLEN=0;			// SLEEP enters sleep mode, section 2.2.4
	OSCCONbits.SCS=0;			// system clock determined by config bits, section 2.3
	OSCTUNEbits.PLLEN=1;

	// initialise the lcd
	lcd_init();	
	lcd_clear();
	lcd_puts(0, "YAPP");
	lcd_puts(1, "SEATALK DISPLAY");
#if defined LAZY_KIPPER
	lcd_puts(3, "V1.1 LAZY KIPPER");
#elif defined SLIPSTREAM
	lcd_puts(3, "V1.1 SLIPSTREAM");
#elif defined CUAN
	lcd_puts(3, "V1.1 CUAN");
#else
	lcd_puts(3, "V1.1 STANDARD");
#endif

	// initialise backlight pwm, pin 21, CCP4
	TRISBbits.TRISB0=1;
	CCPTMRS1bits.C4TSEL=0x02;	// timer 6 selected for CCP4
	PR6=0b11110011;
	T6CON=0b00000101;
	CCPR4L=0b00000010;
	CCP4CON=0b00011100;
	TRISBbits.TRISB0=0;
	
	lamps=0;
	seatalk_message_handler(LAMPS1);
	__delay_ms(200);
	lamps=1;
	seatalk_message_handler(LAMPS2);
	__delay_ms(200);
	lamps=2;
	seatalk_message_handler(LAMPS1);
	__delay_ms(200);
	lamps=3;
	seatalk_message_handler(LAMPS2);
	__delay_ms(200);
	lamps=2;
	seatalk_message_handler(LAMPS1);
	__delay_ms(200);
	lamps=1;
	seatalk_message_handler(LAMPS2);
	__delay_ms(200);
	lamps=0;
	seatalk_message_handler(LAMPS1);
	
	// initialise seatalk
	init_seatalk(seatalk_message_handler);	

	// init 1 second timer
	T2CONbits.TMR2ON=0;			// set timer off, section 13.1
	T2CONbits.T2CKPS=3;         // set prescalar to 16, section 13.1
	T2CONbits.T2OUTPS=0x0f;     // set postscalar to 16, section 13.1
	PR2=0x3f;                   // set timer period, section 13.1
	TMR2=0;                     // set timer initial value, section 13.1
	PIR1bits.TMR2IF=0;          // clear interrupt flag, section 9.5
	PIE1bits.TMR2IE=1;          // enable timer interrupt, section 9.6
	T2CONbits.TMR2ON=1;         // set timer on, section 13.1
}

void seatalk_message_handler(unsigned char message_type)
{								
 	if(message_type==LAMPS1 || message_type==LAMPS2)
	{
		switch(lamps)
		{
		case 0:		// 1%
			CCPR4L=0b00000010;
			CCP4CON=0b00011100;
			break;

		case 1:		// 5%
			CCPR4L=0b00001100;
			CCP4CON=0b00001100;
			break;

		case 2:		// 10%
			CCPR4L=0b00011000;
			CCP4CON=0b00011100;
			break;

		case 3:		// 20%
			CCPR4L=0b00110000;
			CCP4CON=0b00111100;
			break;
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
	
	do_seatalk_read();

	// check tick timer
	if(PIE1bits.TMR2IE && PIR1bits.TMR2IF)
	{
		PIR1bits.TMR2IF=0;
		
		timer_counter++;	
		if(timer_counter==600)
		{
			tick_count++;
			tick=TRUE;
			timer_counter=0;
		}	
	}
}
