#include <p18cxxx.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include "sscanf.h"
#include "gps_repeater.h"
#include "hd44780_16x2_lcd.h"
#include "serial.h"
#include "nmea.h"
#include "sscanf.h"

// Configuration bits are described in section 24.1
// Note: For a complete list of the available config pragmas and their values, in MPLAB
// click Help|Topics then select "PIC18 Config Settings" in the Language Tools section.

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

extern volatile char nmea_messages[NUMBER_NMEA_MESSAGES][NMEA_MESSAGE_MAX_LENGTH+1];
extern volatile unsigned long one_second_tick_count;
extern volatile unsigned char tick;
extern unsigned long position_timestamp;
extern unsigned long cog_sog_timestamp;
extern float latitude_dm;
extern float longitude_dm;
extern float sog;
extern float cog;
extern 	char number_buffer[8];
extern char degree[2];

static void write_coord(float coord, char *buffer, coord_t coord_type);

/* display layout
1234567890123456
89o59.53'N 12.5N
123o59.34'W 123o
*/

void main(void) 
{ 		
	unsigned char i;
	char text_line[17];

	// all intializations collected together here
	init_app();

	lcd_puts(0, 6, "YAPP");
	lcd_puts(1, 2, "GPS REPEATER");
	__delay_ms(2000);
	lcd_clear();
	lcd_puts(0, 2, "NO GPS DATA");

	while(1)
	{		
		// process waiting nmea messages
		for(i=0; i<NUMBER_NMEA_MESSAGES; i++)
		{
		    // check if next message has been fully received and ready for processing
			if(nmea_messages[i][0]==MS_READY)
			{
			    // set message state to reading to stop it being overwritten
				nmea_messages[i][0]=MS_READING;
				
				// check NMEA checksum
				if(test_nmea_checksum((char *)(nmea_messages[i]+1)))
				{
					if(strncmppgm2ram((char *)(nmea_messages[i]+4), "RMC,", 3)==0)
					{
						process_rmc_message(i);	
					}
					else if(strncmppgm2ram((char *)(nmea_messages[i]+4), "VTG,", 3)==0)
					{
						process_vtg_message(i);	
					}
					else if(strncmppgm2ram((char *)(nmea_messages[i]+4), "GLL,", 3)==0)
					{
						process_gll_message(i);	
					}
					else if(strncmppgm2ram((char *)(nmea_messages[i]+4), "GGA,", 3)==0)
					{
						process_gga_message(i);	
					}
				}	
				
				// indicate that this message slot is ready to be written to again
				nmea_messages[i][0]=MS_DONE;	
			}	
		}	

		if(tick)
		{
			tick=FALSE;
			lcd_clear();

			if(one_second_tick_count-cog_sog_timestamp<10 && one_second_tick_count-position_timestamp>10)
			{
				// plot cog/sog but not position

				// top line
				strcpypgm2ram(text_line, "NO LAT/LNG " );
				if(sog<10.0f)
				{
					strcatpgm2ram(text_line, " ");
				}			
				if(sog<100.0f)
				{
					ftoa(sog, number_buffer, 1);
					strcat(text_line, number_buffer);
					strcatpgm2ram(text_line, "K");
				}
				lcd_puts_ram(0, 0, text_line);

				// bottom line
				strcpypgm2ram(text_line, " RECEIVED   " );
				itoa((unsigned int)cog, number_buffer);
				strcat(text_line, number_buffer);
				strcat(text_line, degree);
				lcd_puts_ram(1, 0, text_line);
			}
			else if(one_second_tick_count-cog_sog_timestamp>10 && one_second_tick_count-position_timestamp<10)
			{
				// plot position but not cog/sog

				// top line
				write_coord(latitude_dm, text_line, LATITUDE);
				strcatpgm2ram(text_line, "--.-K");
				lcd_puts_ram(0, 0, text_line);
		
				// bottom line
				write_coord(longitude_dm, text_line, LONGITUDE);
				strcatpgm2ram(text_line, "---");
				strcat(text_line, degree);
				lcd_puts_ram(1, 0, text_line);
			}
			else if(one_second_tick_count-cog_sog_timestamp<10 && one_second_tick_count-position_timestamp<10)
			{
				// plot both

				// top line
				write_coord(latitude_dm, text_line, LATITUDE);
	
				if(sog<10.0f)
				{
					strcatpgm2ram(text_line, " ");
				}			
				if(sog<100.0f)
				{
					ftoa(sog, number_buffer, 1);
					strcat(text_line, number_buffer);
					strcatpgm2ram(text_line, "K");
				}
				lcd_puts_ram(0, 0, text_line);
		
				// bottom line
				write_coord(longitude_dm, text_line, LONGITUDE);
				itoa((unsigned int)cog, number_buffer);
				strcat(text_line, number_buffer);
				strcat(text_line, degree);
				lcd_puts_ram(1, 0, text_line);
			}
			else
			{
				lcd_puts(0, 2, "NO GPS DATA");
			}
		}
	}	
}

static void write_coord(float coord, char *buffer, coord_t coord_type)
{
	float minutes;

	buffer[0]=0;
	if(fabs(coord)<10.0f)
	{
		strcatpgm2ram(buffer, " ");
	}
	itoa((unsigned int)(fabs(coord)), number_buffer);
	strcat(buffer, number_buffer);
	strcat(buffer, degree);
	
	minutes=fabs(coord)-(float)(unsigned int)(fabs(coord));
	minutes*=100.0f;
	sprintf(number_buffer, "%02u.%02u", (unsigned int)minutes, (unsigned int)((minutes*100.0f)-((unsigned int)minutes)*100.0f));
	strcat(buffer, number_buffer);
	if(coord<0.0f)
	{
		if(coord_type==LATITUDE)
		{
			strcatpgm2ram(buffer, "'S ");
		}
		else
		{
			strcatpgm2ram(buffer, "'W ");
		}
	}
	else
	{
		if(coord_type==LATITUDE)
		{
			strcatpgm2ram(buffer, "'N ");
		}
		else
		{
			strcatpgm2ram(buffer, "'E ");
		}
	}
}

void init_app(void)
{
	unsigned int i;

	for(i=0; i<NUMBER_NMEA_MESSAGES; i++)
	{
		nmea_messages[i][0]=MS_DONE;
	}

	OSCCONbits.IRCF=5;			// internal oscillator 4MHz, section 2.2.2
	OSCCONbits.IDLEN=0;			// SLEEP enters sleep mode, section 2.2.4
	OSCCONbits.SCS=0;			// system clock determined by config bits, section 2.3
	OSCTUNEbits.PLLEN=1;

	// init 1 second timer
	T2CONbits.TMR2ON=0;			// set timer off, section 13.1
	T2CONbits.T2CKPS=3;         // set prescalar to 16, section 13.1
	T2CONbits.T2OUTPS=0x0f;     // set postscalar to 16, section 13.1
	PR2=0x3f;                   // set timer period, section 13.1
	TMR2=0;                     // set timer initial value, section 13.1
	PIR1bits.TMR2IF=0;          // clear interrupt flag, section 9.5
	PIE1bits.TMR2IE=1;          // enable timer interrupt, section 9.6
	T2CONbits.TMR2ON=1;         // set timer on, section 13.1

	lcd_init();
	serial_setup();

	RCONbits.IPEN=0;	        // disable interrupt priority, section 9.2	
	INTCONbits.GIE=1;           // globally enable interrupts, section 9.4
	INTCONbits.PEIE=1;          // enable all unmasked peripheral interrupts, section 9.4		
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
	static unsigned char current_writing_message=0;
	static unsigned char next_writing_position=0;
	static unsigned char waiting_for_message_to_start=TRUE;
	unsigned char read_byte;
	
	// check tick timer
	if(PIE1bits.TMR2IE && PIR1bits.TMR2IF)
	{
		PIR1bits.TMR2IF=0;
		
		timer_counter++;	
		if(timer_counter==250)
		{
			one_second_tick_count++;
			tick=TRUE;
			timer_counter=0;
		}	
	}

	// check uart1
	if(PIE1bits.RC1IE && PIR1bits.RC1IF)
	{
		read_byte=RCREG1;
		PIR1bits.RC1IF=0;
		
		if(waiting_for_message_to_start)
		{			
			if(read_byte=='$')
			{				
				waiting_for_message_to_start=FALSE;
				current_writing_message++;
				if(current_writing_message==NUMBER_NMEA_MESSAGES)
				{
					current_writing_message=0;
				}	
				if(nmea_messages[current_writing_message][0]==MS_READING)
				{
					current_writing_message++;
					if(current_writing_message==NUMBER_NMEA_MESSAGES)
					{
						current_writing_message=0;
					}	
				}	
				nmea_messages[current_writing_message][1]=read_byte;
				next_writing_position=2;
			}	
		}	
		else
		{			
			if(read_byte=='\r')
			{				
				nmea_messages[current_writing_message][next_writing_position]=0;
				nmea_messages[current_writing_message][0]=MS_READY;
				waiting_for_message_to_start=TRUE;
			}	
			else
			{				
				nmea_messages[current_writing_message][next_writing_position]=read_byte;				
				next_writing_position++;
				if(next_writing_position==NMEA_MESSAGE_MAX_LENGTH)
				{
					waiting_for_message_to_start=TRUE;
				}	
			}
		}	
	}	
}
