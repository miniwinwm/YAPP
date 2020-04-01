#include <p18cxxx.h>
#include <delays.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "HD44780_16x2_lcd.h"
#include "seatalk_repeater_16x2.h"
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

extern unsigned char top_line_changed;
extern unsigned char bottom_line_changed;
extern unsigned long last_top_button_press_time;
extern unsigned long last_bottom_button_press_time;
extern display_t current_display_top;
extern display_t current_display_bottom;
extern char text_line[17]; 
extern message_handler handler;
extern unsigned char backlight;
extern volatile unsigned char seatalk_messages_in[SEATALK_NUMBER_OF_MESSAGES_IN][SEATALK_MAX_MESSAGE_SIZE+1];
extern volatile unsigned long one_second_tick_count;
extern volatile unsigned char tick;
extern volatile unsigned char buttons;
extern volatile unsigned int c; 
extern char number_buffer[8];

extern float seatalk_depth;
extern float seatalk_boatspeed;
extern float seatalk_heading_magnetic;
extern float seatalk_variation;
extern float seatalk_temperature;
extern float seatalk_trip;
extern float seatalk_log;
extern float seatalk_apparent_wind_angle;
extern float seatalk_apparent_wind_speed;
extern float seatalk_sog;
extern float seatalk_cog;
extern signed int seatalk_latitude_degrees;
extern float seatalk_latitude_minutes;
extern signed int seatalk_longitude_degrees;
extern float seatalk_longitude_minutes;
extern time_t seatalk_gmt;
extern date_t seatalk_date;
extern float seatalk_distance_to_destination;
extern float seatalk_bearing_to_destination;

extern unsigned long depth_received_time;
extern unsigned long boatspeed_received_time;
extern unsigned long heading_mag_received_time;
extern unsigned long variation_received_time;
extern unsigned long trip_received_time;
extern unsigned long log_received_time;
extern unsigned long awa_received_time;
extern unsigned long aws_received_time;
extern unsigned long sog_received_time;
extern unsigned long cog_received_time;
extern unsigned long lat_received_time;
extern unsigned long long_received_time;
extern unsigned long time_received_time;
extern unsigned long date_received_time;
extern unsigned long temperature_received_time;
extern unsigned long nav_data_received_time;

static void init_app(void);
static void seatalk_message_handler(unsigned char message_type);
static void clear_watchdog(void);
static void set_backlight_level(unsigned char level);
static void update_displayed_data(void);
static void display_data_line(unsigned char line, display_t data_type);
static void write_coord(float coord_degrees, float coord_minutes, char *buffer, coord_t coord_type);
static unsigned int abs(signed int i);
static void secondstoa(long seconds, char *text);

void low_isr(void);

static const rom unsigned char degree_character[8]={0x04, 0x0a, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00};

void main(void) 
{ 
	// all intializations collected together here
	init_app();

	set_backlight_level(BACKLIGHT_MAX);

	lcd_puts(0, 6, "YAPP");
	lcd_puts(1, 0, "SEATALK DISPLAY");

	__delay_ms(1000);
	set_backlight_level(backlight);

	while(1)
	{
		clear_watchdog();
		
		if(tick)
		{
			update_displayed_data();
			tick=FALSE;
		}
			
		if(buttons)
		{
			if(buttons==TOP_BUTTON_SHORT)
			{
				top_line_changed=TRUE;
				last_top_button_press_time=one_second_tick_count;
				
				current_display_top++;
				if(current_display_top==MENU_COUNT)
				{
					current_display_top=0;
				}	
				
				if(current_display_top==BACKLIGHT_0)
				{
					set_backlight_level(0);
				}
				else if(current_display_top==BACKLIGHT_1)
				{
					set_backlight_level(1);
				}	
				else if(current_display_top==BACKLIGHT_2)
				{
					set_backlight_level(2);
				}
				else if(current_display_top==BACKLIGHT_3)
				{
					set_backlight_level(3);
				}				
				else
				{
					set_backlight_level(backlight);
				}	
				
			}
			else if(buttons==TOP_BUTTON_LONG)
			{
				bottom_line_changed=TRUE;
				last_bottom_button_press_time=one_second_tick_count;
								
				current_display_bottom++;
				if(current_display_bottom==MENU_COUNT)
				{
					current_display_bottom=0;
				}	
				
				if(current_display_bottom==BACKLIGHT_0)
				{
					set_backlight_level(0);
				}
				else if(current_display_bottom==BACKLIGHT_1)
				{
					set_backlight_level(1);
				}	
				else if(current_display_bottom==BACKLIGHT_2)
				{
					set_backlight_level(2);
				}
				else if(current_display_bottom==BACKLIGHT_3)
				{
					set_backlight_level(3);
				}				
				else
				{
					set_backlight_level(backlight);
				}	
				
			}	
			
			update_displayed_data();
			buttons=0;
		}		
		
		if(one_second_tick_count-last_top_button_press_time>KEYPRESS_TIMEOUT && top_line_changed)
		{
			if(current_display_top==BACKLIGHT_0 || current_display_top==BACKLIGHT_1 || current_display_top==BACKLIGHT_2 || current_display_top==BACKLIGHT_3)
			{
				backlight=current_display_top-BACKLIGHT_0;
				int_EEPROM_putc(EEPROM_POS_BACKLIGHT, backlight);
				current_display_top=int_EEPROM_getc(EEPROM_POS_DISPLAY_TOP);
			}	
			else
			{
				int_EEPROM_putc(EEPROM_POS_DISPLAY_TOP, current_display_top);
			}		
			
			top_line_changed=FALSE;
		} 	
		
		if(one_second_tick_count-last_bottom_button_press_time>KEYPRESS_TIMEOUT && bottom_line_changed)
		{
			if(current_display_bottom==BACKLIGHT_0 || current_display_bottom==BACKLIGHT_1 || current_display_bottom==BACKLIGHT_2 || current_display_bottom==BACKLIGHT_3)
			{
				backlight=current_display_bottom-BACKLIGHT_0;
				int_EEPROM_putc(EEPROM_POS_BACKLIGHT, backlight);
				current_display_bottom=int_EEPROM_getc(EEPROM_POS_DISPLAY_BOTTOM);
			}	
			else
			{
				int_EEPROM_putc(EEPROM_POS_DISPLAY_BOTTOM, current_display_bottom);
			}		
			
			bottom_line_changed=FALSE;
		} 

		// process waiting seatalk messages
		seatalk_process_next_message();
	}
}

static unsigned int abs(signed int i)
{
	if(i<0)
	{
		i=-i;
	}
	
	return i;	
}	

static void write_coord(float coord_degrees, float coord_minutes, char *buffer, coord_t coord_type)
{
	float minutes;
	unsigned int number;

	buffer[0]='\0';
	if(fabs(coord_degrees)<10.0f)
	{
		strcatpgm2ram(buffer, "0");
	}

	if(coord_type==LONGITUDE_T)
	{
		if(fabs(coord_degrees)<100.0f)
		{
			strcatpgm2ram(buffer, "0");
		}
	}

	itoa((unsigned int)(fabs(coord_degrees)), number_buffer);
	strcat(buffer, number_buffer);
	strcatpgm2ram(buffer, "\x01");

	number=(unsigned int)coord_minutes;
	if(number<10)
	{
		strcatpgm2ram(buffer, "0");
	}
	itoa(number, number_buffer);
	strcat(buffer, number_buffer);
	strcatpgm2ram(buffer, ".");

	number=(unsigned int)((coord_minutes*100.0f)-((unsigned int)coord_minutes)*100.0f);
	if(number<10)
	{
		strcatpgm2ram(buffer, "0");
	}
	itoa(number, number_buffer);
	strcat(buffer, number_buffer);

	if(coord_degrees<0.0f)
	{
		if(coord_type==LATITUDE_T)
		{
			strcatpgm2ram(buffer, "'S");
		}
		else
		{
			strcatpgm2ram(buffer, "'W");
		}
	}
	else
	{
		if(coord_type==LATITUDE_T)
		{
			strcatpgm2ram(buffer, "'N");
		}
		else
		{
			strcatpgm2ram(buffer, "'E");
		}
	}
}

static void secondstoa(long seconds, char *text)
{
	char temp_text[6];
	long hours, minutes, remaining_seconds;
		
	hours=seconds/3600L;
	minutes=(seconds-hours*3600L)/60L;	
	remaining_seconds=seconds-(minutes*60L+hours*3600L);	
	
	text[0]=0;
	if(hours>0L)
	{	
		ltoa(hours, temp_text);
		strcpy(text, temp_text);
		strcatpgm2ram(text, "h");
	}
	if(minutes>0L || hours>0L)
	{	
		ltoa(minutes, temp_text);
		strcat(text, temp_text);
		strcatpgm2ram(text, "m");
	}
			
	if(hours==0L)
	{
		ltoa(remaining_seconds, temp_text);
		strcat(text, temp_text);								
		strcatpgm2ram(text, "s");
	}		
}

static void display_data_line(unsigned char line, display_t data_type)
{
	unsigned char c;
	unsigned char len;
	int heading_true;
	float x;
	float y;
	float z;
	float t;
	float tws;
	float twa;
	signed int bearing_to_destination_true;
	signed int off_course;
	float vmg_to_destination;
	unsigned long ttg;
	
	text_line[0]='\0';
	switch(data_type)
	{
		case DEPTH:
			if(one_second_tick_count-depth_received_time>MAX_DATA_AGE)
			{
				strcpypgm2ram(text_line, "DEPTH NO DATA");
			}
			else
			{	
				sprintf(text_line, "DEPTH %u.%u m", (unsigned int)seatalk_depth, (unsigned int)(frac(seatalk_depth)*10.0f));
			}
			break;
			
		case APPARENT_WIND:
			if(one_second_tick_count-aws_received_time>MAX_DATA_AGE || one_second_tick_count-awa_received_time>MAX_DATA_AGE || seatalk_apparent_wind_speed>100.0f)
			{
				strcpypgm2ram(text_line, "AWIND NO DATA");
			}
			else
			{	
				//  1234567890123456
				// "AWIND 123oP 99 K"
				if(seatalk_apparent_wind_angle<180.0f)
				{
					sprintf(text_line, "AWIND %u\x01S %u K", (unsigned int)seatalk_apparent_wind_angle, (unsigned int)seatalk_apparent_wind_speed);
				}
				else
				{
					sprintf(text_line, "AWIND %u\x01P %u K", 360-(unsigned int)seatalk_apparent_wind_angle, (unsigned int)seatalk_apparent_wind_speed);					
				}		
			}
			break;
			
		case HEADING_MAGNETIC:
			if(one_second_tick_count-heading_mag_received_time>MAX_DATA_AGE)
			{
				strcpypgm2ram(text_line, "HEAD MAG NO DATA");
			}
			else
			{	
				sprintf(text_line, "HEADING MAG %03u\x01", (unsigned int)seatalk_heading_magnetic);		
			}
			break;
			
		case HEADING_TRUE:
			if(one_second_tick_count-heading_mag_received_time>MAX_DATA_AGE || one_second_tick_count-variation_received_time>MAX_DATA_AGE)
			{
				strcpypgm2ram(text_line, "HEAD TRU NO DATA");
			}
			else
			{	
				heading_true=((signed int)seatalk_variation)+((signed int)seatalk_heading_magnetic);
				if(heading_true>=360)
				{
					heading_true-=360;
				}	
				else if(heading_true<0)
				{
					heading_true-=360;
				}	
				sprintf(text_line, "HEADING TRU %03u\x01", heading_true);		
			}
			break;
			
		case COG:
			if(one_second_tick_count-cog_received_time>MAX_DATA_AGE)
			{
				strcpypgm2ram(text_line, "COG NO DATA");
			}
			else
			{
				sprintf(text_line, "COG %03u\x01", (unsigned int)seatalk_cog);		
			}	
			break;
			
		case SOG:
			if(one_second_tick_count-sog_received_time>MAX_DATA_AGE)
			{
				strcpypgm2ram(text_line, "SOG NO DATA");
			}
			else
			{
				sprintf(text_line, "SOG %u.%u K", (unsigned int)seatalk_sog, (unsigned int)(frac(seatalk_sog)*10.0f));		
			}	
			break;
			
		case LATITUDE:
			if(one_second_tick_count-lat_received_time>MAX_DATA_AGE)
			{
				strcpypgm2ram(text_line, "LAT NO DATA");
			}
			else
			{
				strcpypgm2ram(text_line, "LAT ");
				write_coord((float)seatalk_latitude_degrees, seatalk_latitude_minutes, text_line+4, LATITUDE_T);
			}	
			break;
			
		case LONGITUDE:
			if(one_second_tick_count-long_received_time>MAX_DATA_AGE)
			{
				strcpypgm2ram(text_line, "LONG NO DATA");
			}
			else
			{
				strcpypgm2ram(text_line, "LONG ");
				write_coord((float)seatalk_longitude_degrees, seatalk_longitude_minutes, text_line+5, LONGITUDE_T);
			}	
			break;
			
		case LOG:
			if(one_second_tick_count-log_received_time>MAX_DATA_AGE)
			{
				strcpypgm2ram(text_line, "LOG NO DATA");
			}
			else
			{
				sprintf(text_line, "LOG %lu.%u NM", (unsigned long)seatalk_log, (unsigned int)(frac(seatalk_log)*10.0f));
			}	
			break;
			
		case TRIP:
			if(one_second_tick_count-log_received_time>MAX_DATA_AGE)
			{
				strcpypgm2ram(text_line, "TRIP NO DATA");
			}
			else
			{
				sprintf(text_line, "TRIP %u.%u NM", (unsigned int)seatalk_trip, (unsigned int)(frac(seatalk_trip)*10.0f));
			}	
			break;
			
		case BOATSPEED:
			if(one_second_tick_count-boatspeed_received_time>MAX_DATA_AGE)
			{
				strcpypgm2ram(text_line, "BOATSPEED NO DATA");
			}
			else
			{
				//  1234567890123456
				// "BOATSPEED 12.3 K"
				sprintf(text_line, "BOATSPEED %u.%u K", (unsigned int)seatalk_boatspeed, (unsigned int)(frac(seatalk_boatspeed)*10.0f));
			}	
			break;
			
		case TEMPERATURE:
			if(one_second_tick_count-temperature_received_time>MAX_DATA_AGE)
			{
				strcpypgm2ram(text_line, "TEMP NO DATA");
			}
			else
			{
				sprintf(text_line, "TEMP %d.%u\x01", (int)seatalk_temperature, abs((unsigned int)(frac(seatalk_temperature)*10.0f)));
				strcatpgm2ram(text_line, "C");
			}	
			break;
			
		case TIME_DATE:
			// 1234567890123456
			// 12:34 25/12/14
			if(one_second_tick_count-time_received_time>MAX_DATA_AGE && one_second_tick_count-date_received_time>MAX_DATA_AGE )
			{
				strcpypgm2ram(text_line, "TIMEDATE NO DATA");
			}
			else if(one_second_tick_count-time_received_time<MAX_DATA_AGE && one_second_tick_count-date_received_time>MAX_DATA_AGE)
			{
				sprintf(text_line, "%u:%02u", seatalk_gmt.hour, seatalk_gmt.minute);
			}
			else if(one_second_tick_count-time_received_time>MAX_DATA_AGE && one_second_tick_count-date_received_time<MAX_DATA_AGE)
			{
				sprintf(text_line, "%u/%02u/%u", seatalk_date.date, seatalk_date.month, seatalk_date.year);
			}		
			else
			{
				sprintf(text_line, "%u:%02u %u/%02u/%u", seatalk_gmt.hour, seatalk_gmt.minute, seatalk_date.date, seatalk_date.month, seatalk_date.year);				
			}
			break;		
			
		case TRUE_WIND:
			if(one_second_tick_count-boatspeed_received_time>MAX_DATA_AGE || one_second_tick_count-awa_received_time>MAX_DATA_AGE || one_second_tick_count-aws_received_time>MAX_DATA_AGE)
			{			
				strcpypgm2ram(text_line, "TWIND NO DATA");
			}
			else
			{
				if(seatalk_boatspeed<0.01f)
				{
					tws=seatalk_apparent_wind_speed;
					twa=seatalk_apparent_wind_angle;
				}
				else
				{
					x=seatalk_boatspeed*sin(seatalk_apparent_wind_angle/DEGREES_TO_RADIANS);
					y=seatalk_boatspeed*cos(seatalk_apparent_wind_angle/DEGREES_TO_RADIANS);
					z=seatalk_apparent_wind_speed-y;
					tws=sqrt(z*z+x*x);
					if(tws==0.0f)
					{
						twa=0.0f;
					}
					else
					{
						t=(M_PI_2-seatalk_apparent_wind_angle/DEGREES_TO_RADIANS)+acos(x/tws);
						twa=M_PI-t;
					}
					twa*=DEGREES_TO_RADIANS;
					if(twa<0.0f)
					{
						twa+=360.0f;
					}
				}		
				
				//  1234567890123456
				// "TWIND 123oP 99 K"
				if(twa<180.0f)
				{
					sprintf(text_line, "TWIND %u\x01S %u K", (unsigned int)twa, (unsigned int)tws);
				}
				else
				{
					sprintf(text_line, "AWIND %u\x01P %u K", 360-(unsigned int)twa, (unsigned int)tws);					
				}				
			}	
			break;
			
		case VARIATION:
			if(one_second_tick_count-variation_received_time>MAX_DATA_AGE)
			{
				strcpypgm2ram(text_line, "VARIATN NO DATA");
			}
			else
			{
				if(seatalk_variation>=0.0f)
				{
					sprintf(text_line, "VARIATION %u\x01 E", (unsigned int)seatalk_variation);
				}
				else
				{
					sprintf(text_line, "VARIATION %u\x01 W", (unsigned int)(-seatalk_variation));
				}		
			}	
			break;

		case BEAR_TO_DEST:
			if(one_second_tick_count-nav_data_received_time>MAX_DATA_AGE)
			{
				strcpypgm2ram(text_line, "B to Dst NO DATA");
			}
			else
			{
				sprintf(text_line, "B to DEST %03u\x01", (unsigned int)seatalk_bearing_to_destination);				
			}	
			break;
			
		case DIST_TO_DEST:
			if(one_second_tick_count-nav_data_received_time>MAX_DATA_AGE || seatalk_distance_to_destination>9999.9f)
			{
				strcpypgm2ram(text_line, "D to Dst NO DATA");
			}
			else
			{
				// 1234567890123456
				// D to DEST 1234.5
				sprintf(text_line, "D to DEST %u.%u", (unsigned int)seatalk_distance_to_destination, (unsigned int)(frac(seatalk_distance_to_destination)*10.0f));				
			}	
			break;
			
		case TTG:
			if(one_second_tick_count-nav_data_received_time>MAX_DATA_AGE || 
					one_second_tick_count-variation_received_time>MAX_DATA_AGE ||
					one_second_tick_count-cog_received_time>MAX_DATA_AGE ||
					one_second_tick_count-sog_received_time>MAX_DATA_AGE)
			{
				strcpypgm2ram(text_line, "TTG NO DATA");
			}
			else
			{
				bearing_to_destination_true=seatalk_variation+seatalk_bearing_to_destination;
				off_course=bearing_to_destination_true-seatalk_cog;
				if(off_course<0)
				{
					off_course=-off_course;
				}
				if(off_course>180)
				{
					off_course=360-off_course;
				}

				vmg_to_destination=cos(off_course/DEGREES_TO_RADIANS)*seatalk_sog;
				if(vmg_to_destination>0.0f)
				{
					// calculate time in seconds
					ttg=(unsigned long)((seatalk_distance_to_destination*3600.0f)/vmg_to_destination);
				}
				else
				{
					ttg=0UL;
				}

				strcpypgm2ram(text_line, "TTG ");
				secondstoa((long)ttg, text_line+4);
			}
			break;
			
		case VMG:
			if(one_second_tick_count-nav_data_received_time>MAX_DATA_AGE || 
					one_second_tick_count-variation_received_time>MAX_DATA_AGE ||
					one_second_tick_count-cog_received_time>MAX_DATA_AGE ||
					one_second_tick_count-sog_received_time>MAX_DATA_AGE)
			{
				strcpypgm2ram(text_line, "VMG NO DATA");
			}
			else
			{
				bearing_to_destination_true=seatalk_variation+seatalk_bearing_to_destination;
				off_course=bearing_to_destination_true-seatalk_cog;
				if(off_course<0)
				{
					off_course=-off_course;
				}
				if(off_course>180)
				{
					off_course=360-off_course;
				}

				vmg_to_destination=cos(off_course/DEGREES_TO_RADIANS)*seatalk_sog;
				
				sprintf(text_line, "VMG %d.%u K", (signed int)vmg_to_destination, (unsigned int)(frac(fabs(vmg_to_destination))*10.0f));				
			}
			break;
			
		case BACKLIGHT_0:	
		case BACKLIGHT_1:
		case BACKLIGHT_2:
		case BACKLIGHT_3:
			sprintf(text_line, "BACKLIGHT %d", data_type-BACKLIGHT_0);
			break;			
	}	
	
	len=strlen(text_line);
	for(c=len; c<16; c++)
	{
		text_line[c]=' ';
	}	
	text_line[16]=0;
	
	lcd_puts_ram(line, 0, text_line);
}	

static void update_displayed_data(void)
{
	display_data_line(0, current_display_top);
	display_data_line(1, current_display_bottom);
}	

static void clear_watchdog(void)
{
	ClrWdt();
}	

static void seatalk_message_handler(unsigned char message_type)
{		
	switch(message_type)
	{
		case SEATALK_ID_DEPTH:
			depth_received_time=one_second_tick_count;
			break;

		case SEATALK_ID_SOG:
			sog_received_time=one_second_tick_count;
			break;
			
		case SEATALK_ID_COG:
			cog_received_time=one_second_tick_count;
			break;
			
		case SEATALK_ID_LATITUDE:
			lat_received_time=one_second_tick_count;
			break;
			
		case SEATALK_ID_LONGITUDE:
			long_received_time=one_second_tick_count;
			break;
			
		case SEATALK_ID_APPARENT_WIND_ANGLE:
			awa_received_time=one_second_tick_count;
			break;
			
		case SEATALK_ID_APPARENT_WIND_SPEED:
			aws_received_time=one_second_tick_count;
			break;

		case SEATALK_ID_HEADING_MAGNETIC:
			heading_mag_received_time=one_second_tick_count;
			break;		
			
		case SEATALK_ID_VARIATION:
			variation_received_time=one_second_tick_count;
			break;
			
		case SEATALK_ID_LOG:
			log_received_time=one_second_tick_count;
			break;
			
		case SEATALK_ID_TRIP:
			trip_received_time=one_second_tick_count;
			break;
			
		case SEATALK_ID_BOATSPEED:
			boatspeed_received_time=one_second_tick_count;
			break;
			
		case SEATALK_ID_TEMPERATURE:
			temperature_received_time=one_second_tick_count;
			break;			
			
		case SEATALK_ID_GMT:
			time_received_time=one_second_tick_count;
			break;
			
		case SEATALK_ID_DATE:
			date_received_time=one_second_tick_count;
			break;
			
		case SEATALK_ID_NAV_TO_WAYPOINT:
			nav_data_received_time=one_second_tick_count;
			break;			
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

	lcd_init();
	lcd_set_cg_character(1, degree_character);
	
	// setup timer 2, this is used for the debounce and short/long press timer
	T2CONbits.TMR2ON=0;			// set timer off
	T2CONbits.T2CKPS=3;         // set prescalar to 16
	T2CONbits.T2OUTPS=0x0f;     // set postscalar to 16
	PR2=0xff;                   // set timer period
	TMR2=0;                     // set timer initial value
	PIR1bits.TMR2IF=0;          // clear interrupt flag
	PIE1bits.TMR2IE=0;          // disable timer interrupt
	T2CONbits.TMR2ON=1;         // set timer on

    // setup timer 4, this is used for the backlight 
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

    // setup capture/compare 3, this is used for the backlight pwm output
    TRISCbits.TRISC6=0;  		
	CCPTMRS0bits.C3TSEL=1;		// timer 4
    CCPR3L=0x88;                // set the beeper duty cycle to 0% 
    CCP3CONbits.DC3B=0;       	// the 2 lsb of the duty cycle, section 15.4.2, register 15-1
    CCP3CONbits.CCP3M=0xf;      // set CCP3 to PWM mode, register 15-1

	// buttons
	TRISBbits.TRISB4=1;
	WPUBbits.WPUB4=1;
	INTCON2bits.RBPU=0;			// enable all port b pull ups
	IOCBbits.IOCB4=1;
	INTCONbits.RBIF=0;			// clear port b change interrupt

	// seatalk
	init_seatalk(seatalk_message_handler);

	// init the eeprom
	if(int_EEPROM_getc(0)!='Y' || int_EEPROM_getc(1)!='A' || int_EEPROM_getc(2)!='P' || int_EEPROM_getc(3)!='P')
	{
        // nothing or corrupt data in the eeprom so initialize it
		int_EEPROM_putc(4, 0);		// version
		int_EEPROM_putc(EEPROM_POS_BACKLIGHT, backlight);
		int_EEPROM_putc(EEPROM_POS_DISPLAY_TOP, current_display_top);
		int_EEPROM_putc(EEPROM_POS_DISPLAY_BOTTOM, current_display_bottom);
		int_EEPROM_putc(0, 'Y');
		int_EEPROM_putc(1, 'A');
		int_EEPROM_putc(2, 'P');
		int_EEPROM_putc(3, 'P');
	}
	else
	{
        // eeprom marker ok so read settings
		backlight=int_EEPROM_getc(EEPROM_POS_BACKLIGHT);	
		current_display_top=int_EEPROM_getc(EEPROM_POS_DISPLAY_TOP);	
		current_display_bottom=int_EEPROM_getc(EEPROM_POS_DISPLAY_BOTTOM);	
	}

	// turn on interrupts
	RCONbits.IPEN=0;	        // disable interrupt priority, section 9.2
    PIE5bits.TMR6IE=1;          // enable timer6 interrupt 
	INTCONbits.RBIE=1;			// enable port b change interrupts
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
		if((PORTBbits.RB4) && !PIE1bits.TMR2IE)		
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
		if(c==12)					
		{
			if(PORTBbits.RB4)			// is input pin still low?
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
		else if(c>12 && c<60)
		{
			if(!PORTBbits.RB4)
			{
				// got a short press
				buttons=temp_buttons;		// signal main loop that a button press has happened
				INTCONbits.RBIE=1;			// enable port b interrupts again
				PIE1bits.TMR2IE=0;
			}	
		}		
		else if(c>=60)
		{
			// got a long press
			buttons=temp_buttons+4;			// signal main loop that a long button press has happened
			INTCONbits.RBIE=1;				// enable port b interrupts again	
			PIE1bits.TMR2IE=0;		
		}
	}

	// check tick timer
	if(PIE5bits.TMR6IE && PIR5bits.TMR6IF)
	{
		PIR5bits.TMR6IF=0;

		do_seatalk_read();
		
		timer_counter++;	
		if(timer_counter==38400)
		{
			one_second_tick_count++;
			tick=TRUE;
			timer_counter=0;
		}	
	}	
}
