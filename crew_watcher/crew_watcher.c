#include <p18cxxx.h>
#include <delays.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "nrf24l01.h"
#include "HD44780_16x2_lcd.h"
#include "serial.h"
#include "nmea.h"
#include "crew_watcher.h"
#include "sscanf.h"
#include "eeprom.h"
#include "mathematics.h"
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

extern volatile char nmea_messages[NUMBER_NMEA_MESSAGES][NMEA_MESSAGE_MAX_LENGTH+1];
extern volatile unsigned char rf_read_ready;
extern volatile unsigned long one_second_tick_count;
extern volatile unsigned char tick;
extern char text_line[17];
extern unsigned long latitude_timestamp;
extern unsigned long longitude_timestamp;
extern unsigned long last_alarm_accept_time;
extern float latitude_degrees;
extern float longitude_degrees;
extern float latitude_minutes;
extern float longitude_minutes;
extern 	char number_buffer[8];
extern char degree[2];
extern volatile unsigned char buttons;
extern volatile unsigned int c; 
extern unsigned char channel_times[CHANNELS];
extern unsigned char channel_config[CHANNELS];
extern unsigned char alarm_time;
extern app_state_t app_state;
extern unsigned char alarm_channel;
extern float alarm_latitude_degrees;
extern float alarm_longitude_degrees;
extern float alarm_latitude_minutes;
extern float alarm_longitude_minutes;
extern unsigned char alarm_pos_available;
extern settings_state_t settings_state;
extern unsigned char main_menu_position;
extern unsigned char channel_menu_position;
extern unsigned char backlight;
extern volatile unsigned char seatalk_messages[SEATALK_NUMBER_OF_MESSAGES][SEATALK_MAX_MESSAGE_SIZE+1];
extern unsigned char position_display_time;
extern unsigned char settings_display_time;
extern unsigned char seatalk_sentence[18];
extern volatile unsigned char seatalk_byte_to_write;
extern volatile unsigned char seatalk_command_bit;
extern volatile unsigned char seatalk_transmit_state;
extern switch_state_t alarm_output_normal_state;

static void init_app(void);
static void show_monitoring_display(void);
static void show_position_display(void);
static void show_alarm_display(void);
static void show_settings_display(void);
static void show_confirm_cancel_alarm_display(void);
static void check_rf_incoming_messages(void);
static void process_nmea_messages(void);
static void do_monitoring_state(void);
static void do_alarm_state(void);
static void do_confirm_cancel_alarm_state(void);
static void do_settings_state(void);
static void do_position_state(void);
static void set_backlight_level(unsigned char level);
static void seatalk_message_handler(unsigned char message_type);
static void write_coord(float coord_degrees, float coord_minutes, char *buffer, coord_t coord_type);
static void cancel_alarm(void);
static void clear_watchdog(void);
static void cancel_alarm_test(void);
static void pulse_backlight(void);
static void enable_external_output(unsigned char enabled);
static void write_seatalk_sentence_with_retries(unsigned char length, unsigned char* command, unsigned char tries);

void low_isr(void);

static const rom unsigned char gps_character[8]={0, 21, 21, 14, 4, 4, 4, 4};
static const rom unsigned char no_gps_character[8]={0, 0, 2, 2, 2, 2, 0, 2};
static const rom unsigned char degree_character[8]={4, 10, 4, 0, 0, 0, 0, 0};
static const rom char main_menu_1[]="CHANNELS";
static const rom char main_menu_2[]="ALARM TIME";
static const rom char main_menu_3[]="BACKLIGHT";
static const rom char main_menu_4[]="OUTPUT";
static const rom char main_menu_5[]="ALARM TEST";
static const rom char *main_menu_items[]={main_menu_1, main_menu_2, main_menu_3, main_menu_4, main_menu_5};
#define MAIN_MENU_COUNT (sizeof(main_menu_items)/sizeof(const rom char *))
static const rom char channel_menu_1[]="CHANNEL 1";
static const rom char channel_menu_2[]="CHANNEL 2";
static const rom char channel_menu_3[]="CHANNEL 3";
static const rom char channel_menu_4[]="CHANNEL 4";
static const rom char *channel_menu_items[]={channel_menu_1, channel_menu_2, channel_menu_3, channel_menu_4};
#define CHANNEL_MENU_COUNT (sizeof(channel_menu_items)/sizeof(const rom char *))

void main(void) 
{ 
	// all intializations collected together here
	init_app();

	lcd_puts(0, 6, "YAPP");
	lcd_puts(1, 2, "CREW WATCHER");

	TRISBbits.TRISB0=0;		// beeper on  
	__delay_ms(150);
	TRISBbits.TRISB0=1;		// beeper off
	__delay_ms(650);
		
	while(1)
	{
		clear_watchdog();
		
		// check to see if anything has come in via RF
		check_rf_incoming_messages();

		// process waiting nmea messages
		process_nmea_messages();

		// process waiting seatalk messages
		parse_next_seatalk_message();

		switch(app_state)
		{
			case MONITORING:
				do_monitoring_state();
				break;

			case ALARM:
				do_alarm_state();
				break;
				
			case CONFIRM_CANCEL_ALARM:
				do_confirm_cancel_alarm_state();
				break;

			case SETTINGS:
				do_settings_state();
				break;

			case POSITION:
				do_position_state();
				break;
		}
	}
}

static void clear_watchdog(void)
{
	ClrWdt();
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
	switch(message_type)
	{
		case LATITUDE:
			latitude_timestamp=one_second_tick_count;
			break;

		case LONGITUDE:
			longitude_timestamp=one_second_tick_count;
			break;
			
		case CANCEL_MOB:
			TRISBbits.TRISB0=1;		// beeper off  
			enable_external_output(FALSE);
			cancel_alarm();
			break;
	}
}

static void do_position_state(void)
{
	if(tick)
	{
		tick=FALSE;
		position_display_time++;
		if(position_display_time==10)
		{
			app_state=MONITORING;
			show_monitoring_display();
			buttons=0;
			return;
		}
		
		show_position_display();		
	}
	
	if(buttons==TOP_BUTTON)
	{
		app_state=MONITORING;
		show_monitoring_display();
		buttons=0;
		return;
	}
}

static void do_monitoring_state(void)
{
	unsigned char i;

	if(buttons==TOP_BUTTON)
	{
		app_state=SETTINGS;
		settings_state=MAIN;
		main_menu_position=0;
		settings_display_time=0;
		show_settings_display();
		buttons=0;
	}
	else if(buttons==BOTTOM_BUTTON)
	{
		app_state=POSITION;
		show_position_display();
		position_display_time=0;
		buttons=0;
	}

	if(tick)
	{
		tick=FALSE; 

		for(i=0; i<CHANNELS; i++)
		{
			if(channel_config[i] & 0x01 && channel_times[i]<100)
			{
				channel_times[i]++;
			}

			// check if an alarm need raising
			if(app_state==MONITORING && 
					channel_times[i]>alarm_time && 
					one_second_tick_count-last_alarm_accept_time>ALARM_REARM_TIME_SECONDS)
			{
				// goto alarm state
				app_state=ALARM;
				if(one_second_tick_count-latitude_timestamp<GPS_TIMEOUT_SECONDS &&
						one_second_tick_count-longitude_timestamp<GPS_TIMEOUT_SECONDS)
				{
					alarm_latitude_degrees=latitude_degrees;
					alarm_longitude_degrees=longitude_degrees;
					alarm_latitude_minutes=latitude_minutes;
					alarm_longitude_minutes=longitude_minutes;
					alarm_pos_available=TRUE;
				}
				else
				{
					alarm_pos_available=FALSE;
				}
				TRISBbits.TRISB0=0;		// beeper on  
				enable_external_output(TRUE);
				alarm_channel=i+1;		
				
				// send seatalk mob commands
				memcpypgm2ram((char *)seatalk_sentence, (const far rom void *)"\x82\xA5\x40\xBF\x92\x6D\x24\xDB", 8);
				write_seatalk_sentence_with_retries(8, seatalk_sentence, 5);
				memcpypgm2ram((char *)seatalk_sentence, (const far rom void *)"\x6E\x07\x00\x00\x00\x00\x00\x00\x00\x00", 10);
				write_seatalk_sentence_with_retries(10, seatalk_sentence, 5);						
				
				show_alarm_display();
				return;
			}
		}

		show_monitoring_display();
	}
}

static void do_alarm_state(void)
{
	if(tick)
	{
		tick=FALSE;
		show_alarm_display();
		pulse_backlight();	
	}

	if(buttons>0)
	{
		TRISBbits.TRISB0=1;		// beeper off
		enable_external_output(FALSE);

		if(buttons==BOTTOM_BUTTON)
		{
			app_state=CONFIRM_CANCEL_ALARM;
			show_confirm_cancel_alarm_display();			
		}
		buttons=0;	  
	}
}

static void do_confirm_cancel_alarm_state(void)
{
	if(tick)
	{
		tick=FALSE;
	}
	
	if(buttons>0)
	{
		if(buttons==TOP_BUTTON)
		{
			cancel_alarm();
			
			// send seatalk cancel mob alarm
			memcpypgm2ram((char *)seatalk_sentence, (const far rom void *)"\x36\x00\x01", 3);
			write_seatalk_sentence_with_retries(3, seatalk_sentence, 5);			
		}
		else
		{
			app_state=ALARM;
			show_alarm_display();
		}	
		buttons=0;	  
	}
}	

static void show_confirm_cancel_alarm_display(void)
{
	lcd_clear();
	lcd_puts(0, 0, "CONFIRM?     YES"); 
	lcd_puts(1, 14, "NO");
}	

static void cancel_alarm(void)
{
	unsigned char i;
	
	for(i=0; i<CHANNELS; i++)
	{
		channel_times[i]=0;		
	}	
	
	app_state=MONITORING;
	show_monitoring_display();
	alarm_pos_available=FALSE;
	last_alarm_accept_time=one_second_tick_count;
	set_backlight_level(backlight);
}	

static void cancel_alarm_test(void)
{
	TRISBbits.TRISB0=1;		// beeper off  
	enable_external_output(FALSE);	
	set_backlight_level(backlight);		// backlight back to normal		
	app_state=MONITORING;
	show_monitoring_display();	
}	

static void pulse_backlight(void)
{
	if(0x01&(unsigned char)one_second_tick_count)
	{
		set_backlight_level(backlight);	
	}	
	else
	{
		if(backlight<BACKLIGHT_MAX)
		{
			set_backlight_level(backlight+1);
		}	
		else
		{
			set_backlight_level(BACKLIGHT_MAX-1);
		}	
	}			
}	

static void do_settings_state(void)
{
	if(tick)
	{
		tick=FALSE;
		
		// check if test alarm is happening (beeper on) and pulse backlight if it is
		if(TRISBbits.TRISB0==0)
		{
			pulse_backlight();
		}
			
		settings_display_time++;
		if(settings_display_time==10)
		{
			// cancel alarm test in case it was active
			cancel_alarm_test();
			buttons=0;
			return;
		}
	}
	
	if(buttons>0)
	{
		settings_display_time=0;
		
		if(settings_state==MAIN)
		{
			if(buttons==TOP_BUTTON)
			{
				main_menu_position++;
				if(main_menu_position==MAIN_MENU_COUNT)
				{
					// cancel alarm test in case it was active
					cancel_alarm_test();					
					buttons=0;						
					return;
				}
			}
			else if(buttons==BOTTOM_BUTTON)
			{
				switch(main_menu_position)
				{
					case 0:
						settings_state=CHANNEL_SETUP;
						channel_menu_position=0;
						break;
	
					case 1:
						settings_state=ALARM_TIME;
						break;

					case 2:
						settings_state=BACKLIGHT;
						break;
						
					case 3:
						settings_state=ALARM_OUTPUT;
						break;
						
					case 4:
						TRISBbits.TRISB0=0;		// beeper on  
						enable_external_output(TRUE);						
						break;
				}
			}
		}
		else if(settings_state==ALARM_TIME)
		{
			if(buttons==TOP_BUTTON)
			{
				alarm_time++;
				if(alarm_time>ALARM_TIME_MAX)
				{
					alarm_time=ALARM_TIME_MIN;
				}
			}
			else if(buttons==BOTTOM_BUTTON)
			{
				int_EEPROM_putc(10, alarm_time);
				show_monitoring_display();
				app_state=MONITORING;
			}
		}
		else if(settings_state==CHANNEL_SETUP)
		{
			if(buttons==TOP_BUTTON)
			{
				if(channel_menu_position==CHANNEL_MENU_COUNT-1)
				{
					// exit
					show_monitoring_display();
					app_state=MONITORING;
				}
				else
				{
					channel_menu_position++;
				}
			}
			else if(buttons==BOTTOM_BUTTON)
			{
				channel_config[channel_menu_position]^=0x01;
				int_EEPROM_putc(6+channel_menu_position, channel_config[channel_menu_position]);			
			}
		}
		else if(settings_state==BACKLIGHT)
		{
			if(buttons==TOP_BUTTON)
			{
				backlight++;
				if(backlight>BACKLIGHT_MAX)
				{
					backlight=0;
				}
				set_backlight_level(backlight);
			}
			else if(buttons==BOTTOM_BUTTON)
			{
				int_EEPROM_putc(11, backlight);
				show_monitoring_display();
				app_state=MONITORING;				
			}
		}
		else if(settings_state==ALARM_OUTPUT)
		{
			if(buttons==TOP_BUTTON)
			{
				alarm_output_normal_state=!alarm_output_normal_state;
				enable_external_output(FALSE);
			}
			else if(buttons==BOTTOM_BUTTON)
			{
				int_EEPROM_putc(12, alarm_output_normal_state);	
				show_monitoring_display();
				app_state=MONITORING;
			}			
		}	
		buttons=0;	  

		show_settings_display();
	}
}

static void show_position_display(void)
{
	lcd_clear();
	if(one_second_tick_count-latitude_timestamp<GPS_TIMEOUT_SECONDS &&
			one_second_tick_count-longitude_timestamp<GPS_TIMEOUT_SECONDS)
	{
		write_coord(latitude_degrees, latitude_minutes, text_line, LATITUDE_T);
		lcd_puts_ram(0, 0, text_line);
	
		write_coord(longitude_degrees, longitude_minutes, text_line, LONGITUDE_T);
		lcd_puts_ram(1, 0, text_line);
		}
	else
	{
		lcd_puts(0, 0, "NO GPS");
	}

	lcd_puts(0, 12, "EXIT");
}

static void show_settings_display(void)
{
	lcd_clear();
	
	switch(settings_state)
	{
		case MAIN:
			lcd_puts(0, 0, main_menu_items[main_menu_position]);
			if(main_menu_position==MAIN_MENU_COUNT-1)
			{
				lcd_puts(0, 12, "EXIT");
				if(TRISBbits.TRISB0==1)		// is alarm sound off, i.e. we are not testing alarm yet
				{
					// alarm sound is off so show select
					lcd_puts(1, 10, "SELECT");
				}
				else
				{
					lcd_puts(1, 0, SOFTWARE_VERSION_NUMBE);
				}
			}
			else
			{
				lcd_puts(1, 10, "SELECT");
				lcd_puts(0, 12, "NEXT");
			}	
			break;

		case ALARM_TIME:
			strcpypgm2ram(text_line, (const far rom char *)"TIME=");
			itoa(alarm_time, number_buffer);
			strcat(text_line, number_buffer);
			strcatpgm2ram(text_line, (const far rom char *)"s");
			lcd_puts_ram(0, 0, text_line);
			lcd_puts(0, 12, "INCR");
			lcd_puts(1, 12, "EXIT");
			break;

		case BACKLIGHT:
			strcpypgm2ram(text_line, (const far rom char *)"LEVEL=");
			itoa(backlight, number_buffer);
			strcat(text_line, number_buffer);
			lcd_puts_ram(0, 0, text_line);
			lcd_puts(0, 12, "INCR");
			lcd_puts(1, 12, "EXIT");
			break;
			
		case ALARM_OUTPUT:
			strcpypgm2ram(text_line, (const far rom char *)"OUTPUT=");
			if(alarm_output_normal_state==NORMALLY_OPEN)
			{
				strcatpgm2ram(text_line, (const far rom char *)"N/O");
			}
			else
			{
				strcatpgm2ram(text_line, (const far rom char *)"N/C");				
			}		
			lcd_puts_ram(0, 0, text_line);
			lcd_puts(0, 12, "FLIP");
			lcd_puts(1, 12, "EXIT");	
			break;

		case CHANNEL_SETUP:	
			lcd_puts(0, 0, channel_menu_items[channel_menu_position]);
			if(channel_menu_position==CHANNEL_MENU_COUNT-1)
			{
				lcd_puts(0, 12, "EXIT");
			}
			else
			{
				lcd_puts(0, 12, "NEXT");
			}
			lcd_puts(1, 10, "CHANGE");
			if(channel_config[channel_menu_position])
			{
				lcd_puts(1, 0, "ON");
			}
			else
			{
				lcd_puts(1, 0, "OFF");
			}
			break;		
	}
}

static void show_monitoring_display(void)
{
	unsigned char i;

	text_line[0]=0;
	if(one_second_tick_count-last_alarm_accept_time<ALARM_REARM_TIME_SECONDS)
	{
		strcatpgm2ram(text_line, (const far rom char *)"RE-ARMING ");
		itoa(ALARM_REARM_TIME_SECONDS-(unsigned int)(one_second_tick_count-last_alarm_accept_time), number_buffer);
		strcat(text_line, number_buffer);
		for(i=0; i<CHANNELS; i++)
		{
			channel_times[i]=0;
		}	
	}
	else
	{	
		for(i=0; i<CHANNELS; i++)
		{
			if(channel_config[i] & 0x01)
			{	
				if(channel_times[i]<100)
				{
					if(channel_times[i]<10)
					{
						strcatpgm2ram(text_line, (const far rom char *)" ");
					}
					itoa((unsigned int)channel_times[i], number_buffer);
					strcat(text_line, number_buffer);
					strcatpgm2ram(text_line, (const far rom char *)" ");
				}
				else
				{
					strcatpgm2ram(text_line, (const far rom char *)"?? ");
				}
			}
			else
			{
				strcatpgm2ram(text_line, (const far rom char *)"-- ");
			}
		}
	}

	lcd_clear();
	lcd_puts_ram(1, 0, text_line);
	lcd_puts(0, 0, " 1  2  3  4 MENU");

	// show gps status
	if(one_second_tick_count-latitude_timestamp<GPS_TIMEOUT_SECONDS &&
			one_second_tick_count-longitude_timestamp<GPS_TIMEOUT_SECONDS)
	{
		// gps ok
		lcd_show_cg_character(1, 15, 0);
	}
	else
	{
		// gps not ok
		if(one_second_tick_count & 0x00000001)
		{
			lcd_show_cg_character(1, 15, 1);
		}
	}
}

static void show_alarm_display(void)
{
	unsigned int distance;
	unsigned int bearing;
	float alarm_latitude_dd;
	float alarm_longitude_dd;
	float latitude_dd;
	float longitude_dd;
	float coord_dd;
	unsigned char coord_negative;

	lcd_clear();

	strcpypgm2ram(text_line, (const far rom char *)"ALARM ");
	itoa((unsigned int)alarm_channel, number_buffer);
	strcat(text_line, number_buffer);
	if(TRISBbits.TRISB0==0)
	{
		strcatpgm2ram(text_line, (const far rom char *)"  SILENCE");
	}
	lcd_puts_ram(0, 0, text_line);

	// show gps status
	if(one_second_tick_count-latitude_timestamp<GPS_TIMEOUT_SECONDS &&
			one_second_tick_count-longitude_timestamp<GPS_TIMEOUT_SECONDS && 
			alarm_pos_available)
	{
		// gps ok
		latitude_dd=convert_degrees_and_minutes_to_degrees_frac_degrees(latitude_degrees, latitude_minutes);
		longitude_dd=convert_degrees_and_minutes_to_degrees_frac_degrees(longitude_degrees, longitude_minutes);
		alarm_latitude_dd=convert_degrees_and_minutes_to_degrees_frac_degrees(alarm_latitude_degrees, alarm_latitude_minutes);
		alarm_longitude_dd=convert_degrees_and_minutes_to_degrees_frac_degrees(alarm_longitude_degrees, alarm_longitude_minutes);

		distance=(unsigned int)distance_between_points(alarm_latitude_dd, alarm_longitude_dd, latitude_dd, longitude_dd);
		bearing=(unsigned int)bearing_between_points(latitude_dd, longitude_dd, alarm_latitude_dd, alarm_longitude_dd);
		if(distance==0)
		{
			bearing=0;
		}

		/*
		1234567890123456
		xxxxm 123oT EXIT
		*/

		if(distance<10000)
		{
			if(distance<10)
			{
				strcpypgm2ram(text_line, (const far rom char *)"   ");
			}
			else if(distance<100)
			{
				strcpypgm2ram(text_line, (const far rom char *)"  ");
			}
			else if(distance<1000)
			{
				strcpypgm2ram(text_line, (const far rom char *)" ");
			}
			else
			{
				text_line[0]='\0';
			}
			itoa(distance, number_buffer);
			strcat(text_line, number_buffer);
			strcatpgm2ram(text_line, (const far rom char *)"m ");

			if(bearing<10)
			{
				strcatpgm2ram(text_line, (const far rom char *)"  ");
			}
			else if(bearing<100)
			{
				strcatpgm2ram(text_line, (const far rom char *)" ");
			}
			itoa(bearing, number_buffer);
			strcat(text_line, number_buffer);
			strcatpgm2ram(text_line, (const far rom char *)" T EXIT");
			lcd_puts_ram(1, 0, text_line);
			lcd_show_cg_character(1, 9, 2);
		}
		else
		{
			lcd_puts(1, 0, "DIST>10km   EXIT");
		}
	}
	else
	{
		// gps not ok
		lcd_puts(1, 0, "NO GPS      EXIT");
	}
}

static void write_coord(float coord_degrees, float coord_minutes, char *buffer, coord_t coord_type)
{
	float minutes;
	unsigned int number;

	buffer[0]=0;
	if(fabs(coord_degrees)<10.0f)
	{
		strcatpgm2ram(buffer, (const far rom char *)"0");
	}

	if(coord_type==LONGITUDE_T)
	{
		if(fabs(coord_degrees)<100.0f)
		{
			strcatpgm2ram(buffer, (const far rom char *)"0");
		}
	}

	itoa((unsigned int)(fabs(coord_degrees)), number_buffer);
	strcat(buffer, number_buffer);
	strcat(buffer, degree);

	number=(unsigned int)coord_minutes;
	if(number<10)
	{
		strcatpgm2ram(buffer, (const far rom char *)"0");
	}
	itoa(number, number_buffer);
	strcat(buffer, number_buffer);
	strcatpgm2ram(buffer, (const far rom char *)".");

	number=(unsigned int)((coord_minutes*100.0f)-((unsigned int)coord_minutes)*100.0f);
	if(number<10)
	{
		strcatpgm2ram(buffer, (const far rom char *)"0");
	}
	itoa(number, number_buffer);
	strcat(buffer, number_buffer);

	if(coord_degrees<0.0f)
	{
		if(coord_type==LATITUDE_T)
		{
			strcatpgm2ram(buffer, (const far rom char *)"'S ");
		}
		else
		{
			strcatpgm2ram(buffer, (const far rom char *)"'W ");
		}
	}
	else
	{
		if(coord_type==LATITUDE_T)
		{
			strcatpgm2ram(buffer, (const far rom char *)"'N ");
		}
		else
		{
			strcatpgm2ram(buffer, (const far rom char *)"'E ");
		}
	}
}

static void process_nmea_messages(void)
{
	unsigned char i;

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
				if(strncmppgm2ram((char *)(nmea_messages[i]+4), (const far rom char *)"RMC,", 3)==0)
				{
					process_rmc_message(i);	
				}
				else if(strncmppgm2ram((char *)(nmea_messages[i]+4), (const far rom char *)"GLL,", 3)==0)
				{
					process_gll_message(i);	
				}
				else if(strncmppgm2ram((char *)(nmea_messages[i]+4), (const far rom char *)"GGA,", 3)==0)
				{
					process_gga_message(i);	
				}
			}	
			
			// indicate that this message slot is ready to be written to again
			nmea_messages[i][0]=MS_DONE;	
		}	
	}
}

static void check_rf_incoming_messages(void)
{
	unsigned char data; 

	if(rf_read_ready)
	{
		// there is data
		rf_read_ready=FALSE;

		// read data
		nrf24l01_read_rx_payload(&data, 1); 
		nrf24l01_irq_clear_all(); 

		// check data validity
		if(data<CHANNELS)
		{
			// data ok, act on message
			channel_times[data]=0;
		}
	}
}

static void set_backlight_level(unsigned char level)
{
	switch(level)
	{
		case 0:
			CCPR3L=0x88;
			break;

		case 1:
			CCPR3L=0x7c;
			break;

		case 2:
			CCPR3L=0x74;
			break;

		case 3:
			CCPR3L=0x66;
			break;
	}
}

static void enable_external_output(unsigned char enabled)
{
	if(alarm_output_normal_state==NORMALLY_OPEN)
	{
		LATAbits.LATA1=enabled;		
	}	
	else
	{
		LATAbits.LATA1=!enabled;				
	}	
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
	
	// external alarm output
	TRISAbits.TRISA1=0;			// set external alarm pin as output

	// set up rf module pins
	TRISBbits.TRISB1=1; 		// set irq pin as input
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
	INTCON2bits.INTEDG1=0;
	INTCON3bits.INT1IF=0;
	
	// init the rf chip as transmitter
	nrf24l01_initialize_debug(true, 1, true); 

	lcd_init();
	lcd_set_cg_character(0, gps_character);
	lcd_set_cg_character(1, no_gps_character);
	lcd_set_cg_character(2, degree_character);

	serial_setup();

	// setup timer 2, this is used for the debounce and short/long press timer
	T2CONbits.TMR2ON=0;			// set timer off
	T2CONbits.T2CKPS=3;         // set prescalar to 16
	T2CONbits.T2OUTPS=0x0f;     // set postscalar to 16
	PR2=0xff;                   // set timer period
	TMR2=0;                     // set timer initial value
	PIR1bits.TMR2IF=0;          // clear interrupt flag
	PIE1bits.TMR2IE=0;          // disable timer interrupt
	T2CONbits.TMR2ON=1;         // set timer on

    // setup timer 4, this is used for the beeper 
    T4CONbits.TMR4ON=0;			// disable timer 4
    T4CONbits.T4CKPS=3;         // set prescalar is 16
    PR4=0x7c;                   // set timer4 period
    TMR4=0;                     // set timer4 initial value
    PIR5bits.TMR4IF=0;          // clear interrupt flag
    PIE5bits.TMR4IE=0;          // disable timer4 interrupt 
    T4CONbits.TMR4ON=1;         // enable timer 4

    // setup timer 6, this is used for the tick timer 
    T6CONbits.TMR6ON=0;			// disable timer 6
    T6CONbits.T6CKPS=0;         // set prescalar is 1
    T6CONbits.T6OUTPS=0;     	// set postscalar to 1
    PR6=0xd0;                   // set timer6 period
    TMR6=0;                     // set timer6
    PIR5bits.TMR6IF=0;          // clear interrupt flag
    T6CONbits.TMR6ON=1;         // enable timer 6

    // setup capture/compare 4, this is used for the beeper PWM output
    TRISBbits.TRISB0=1;  		
	CCPTMRS1bits.C4TSEL=1;		// timer 4
    CCPR4L=0x3e;                // set the duty cycle to 50% 
    CCP4CONbits.DC4B=0;       	// the 2 lsb of the duty cycle, section 15.4.2, register 15-1
    CCP4CONbits.CCP4M=0xf;      // set CCP4 to PWM mode, register 15-1

    // setup capture/compare 3, this is used for the backlight pwm output
    TRISCbits.TRISC6=0;  		
	CCPTMRS0bits.C3TSEL=1;		// timer 2
    CCPR3L=0x88;                // set the duty cycle to 0% 
    CCP3CONbits.DC3B=0;       	// the 2 lsb of the duty cycle, section 15.4.2, register 15-1
    CCP3CONbits.CCP3M=0xf;      // set CCP4 to PWM mode, register 15-1

	// buttons
	TRISBbits.TRISB5=1;			// set button pins as inputs
	TRISBbits.TRISB4=1;
	WPUBbits.WPUB5=1;			// pull ups enabled on button pins
	WPUBbits.WPUB4=1;
	INTCON2bits.RBPU=0;			// enable all port b pull ups
	IOCBbits.IOCB5=1;			// interrupt on change enabled on button pins
	IOCBbits.IOCB4=1;
	INTCONbits.RBIF=0;			// clear port b change interrupt

	// seatalk
	init_seatalk(seatalk_message_handler);

	// check values in eeprom and reset if bad
	if(int_EEPROM_getc(6)>1 ||
		int_EEPROM_getc(7)>1 ||
		int_EEPROM_getc(8)>1 ||
		int_EEPROM_getc(9)>1 ||
		int_EEPROM_getc(10)<ALARM_TIME_MIN || int_EEPROM_getc(6)>ALARM_TIME_MAX ||
		int_EEPROM_getc(11)>3 ||
		int_EEPROM_getc(12)>(unsigned char)NORMALLY_OPEN)
	{
		for(i=0; i<CHANNELS; i++)
		{
			int_EEPROM_putc(6+i, 0x00);			
		}
		int_EEPROM_putc(10, ALARM_TIME_MIN);
		int_EEPROM_putc(11, BACKLIGHT_MAX);	
		int_EEPROM_putc(12, (unsigned char)NORMALLY_OPEN);			
    }

	for(i=0; i<CHANNELS; i++)
	{
		channel_config[i]=int_EEPROM_getc(6+i);
	}
	alarm_time=int_EEPROM_getc(10);	
	backlight=int_EEPROM_getc(11);	
	set_backlight_level(backlight);
	alarm_output_normal_state=(switch_state_t)int_EEPROM_getc(12);
	enable_external_output(FALSE);	
	
	// turn on interrupts
	RCONbits.IPEN=0;	        // disable interrupt priority, section 9.2
    PIE5bits.TMR6IE=1;          // enable timer6 interrupt 
	INTCONbits.RBIE=1;			// enable port b change interrupts
	INTCON3bits.INT1IE=1;		// enable rf module irq line interrupt
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
	static unsigned int timer_counter=0;
	static unsigned char current_writing_message=0;
	static unsigned char next_writing_position=0;
	static unsigned char waiting_for_message_to_start=TRUE;
	unsigned char read_byte;
	static unsigned char temp_buttons;

	if(INTCONbits.RBIE && INTCONbits.RBIF)
	{
		// if falling edge and not currently debouncing...
		if((PORTBbits.RB5 || PORTBbits.RB4) && !PIE1bits.TMR2IE)		
		{
			TMR2=0;
			PIE1bits.TMR2IE=1;				// enable timer interrupt
			INTCONbits.RBIE=0;				// disable further port b interrupts in the meantime	
			c=0;							// clear timer count
		}
					
		INTCONbits.RBIF=0;		
	}	

	// check button timer
	if(PIE1bits.TMR2IE && PIR1bits.TMR2IF)
	{
		PIR1bits.TMR2IF=0;
		c++;								// increment count of number of times timer interrupt has fired
		if(c==6)					
		{
			if(PORTBbits.RB5)			// is input pin still low?
			{
				temp_buttons=1;
			}
			else if(PORTBbits.RB4)			// is input pin still low?
			{
				temp_buttons=2;
			}
			else
			{
				// got a bounce or very short press which is ignored
				PIE1bits.TMR2IE=0;			// kill timer interrupts and go back to sleep in main loop
				INTCONbits.RBIE=1;			// enable port b interrupts again
			}			
		}
		else if(c>6)
		{
			if(!PORTBbits.RB5 && !PORTBbits.RB4)
			{
				// got a short press
				buttons=temp_buttons;		// signal main loop that a button press has happened
				INTCONbits.RBIE=1;			// enable port b interrupts again
				PIE1bits.TMR2IE=0;
			}	
		}		
	}

	// check rf interrupt
	if(INTCON3bits.INT1IE && INTCON3bits.INT1IF)
	{
		INTCON3bits.INT1IF=0;
		rf_read_ready=TRUE;
	}

	// check tick timer
	if(PIE5bits.TMR6IE && PIR5bits.TMR6IF)
	{
		PIR5bits.TMR6IF=0;

		do_seatalk_read();
		do_seatalk_write();
		
		timer_counter++;	
		if(timer_counter==38400)
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
