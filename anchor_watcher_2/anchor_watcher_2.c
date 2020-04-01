#include <p18cxxx.h>
#include <delays.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "HD44780_16x2_lcd.h"
#include "anchor_watcher_2.h"
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

extern float latitude_dfd;
extern float longitude_dfd;
extern float distance_m;
extern float latitude_dfd_ref;
extern float longitude_dfd_ref;
extern unsigned int heading_ref;
extern unsigned char wind_alarm_count;
extern unsigned char depth_alarm_count;
extern unsigned char sog_alarm_count;
extern unsigned char distance_alarm_count;
extern unsigned char heading_alarm_count;
extern setup_state_t setup_state;
extern unsigned char main_menu_position;
extern unsigned long earliest_next_alarm_time;
extern t_alarm alarm_type;
extern unsigned char backlight;
extern char number_buffer[8];
extern app_state_t app_state;
extern volatile unsigned long one_second_tick_count;
extern volatile unsigned char tick;
extern volatile unsigned char buttons;
extern volatile unsigned int c; 
extern char text_line[17];
extern volatile unsigned char seatalk_messages[SEATALK_NUMBER_OF_MESSAGES][SEATALK_MAX_MESSAGE_SIZE+1];
extern message_handler handler;
extern float depth;
extern int latitude_degrees;
extern signed int longitude_degrees;
extern float latitude_minutes;
extern float longitude_minutes;
extern float sog;
extern float awa;
extern float aws;
extern unsigned long depth_received_time;
extern unsigned long lat_received_time;
extern unsigned long long_received_time;
extern unsigned long sog_received_time;
extern unsigned long aws_received_time;
extern unsigned long heading_received_time;
extern unsigned long awa_received_time;
extern unsigned int heading;
extern int heading_difference;
extern unsigned char max_distance;
extern unsigned int min_depth;
extern unsigned char max_wind;
extern unsigned char max_sog;
extern unsigned char max_heading;
extern unsigned char distance_enabled;
extern unsigned char depth_enabled;
extern unsigned char wind_enabled;
extern unsigned char sog_enabled;
extern unsigned char heading_enabled;
extern unsigned char rearm_time;

static void init_app(void);
static void seatalk_message_handler(unsigned char message_type);
static void clear_watchdog(void);
static void set_backlight_level(unsigned char level);
static void pulse_backlight(void);
static int abs(int x);
static int get_heading_error(int initial, int final);
static unsigned char check_data_availability(void);

static void do_waiting_for_data_state(void);
static void show_waiting_for_data_display(void);
static void do_standby_state(void);
static void show_standby_display(void);
static void do_watching_state(void);
static void show_watching_display(void);
static void do_alarm_state(void);
static void show_alarm_display(void);
static void do_setup_state(void);
static void show_setup_display(void);
static void enter_setup_state(void);

void low_isr(void);

static const rom unsigned char degree_character[8]={0x04, 0x0a, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00};
static const rom unsigned char turn_character[8]={0x0e, 0x06, 0x0a, 0x10, 0x10, 0x11, 0x0e, 0x00};
static const rom unsigned char move_character[8]={0x04, 0x0e, 0x1f, 0x04, 0x04, 0x1f, 0x0e, 0x04};

static const rom char main_menu_1[]="DEPTH";
static const rom char main_menu_2[]="SOG";
static const rom char main_menu_3[]="WIND SPEED";
static const rom char main_menu_4[]="HEADING \x02";
static const rom char main_menu_5[]="POSITION \x03";
static const rom char main_menu_6[]="BACKLIGHT";
static const rom char main_menu_7[]="REARM TIME";
static const rom char main_menu_8[]="ALARM TEST";
static const rom char *main_menu_items[]={main_menu_1, main_menu_2, main_menu_3, main_menu_4, main_menu_5, main_menu_6, main_menu_7, main_menu_8};
#define MAIN_MENU_COUNT (sizeof(main_menu_items)/sizeof(const rom char *))

void main(void) 
{ 
	// all intializations collected together here
	init_app();

	set_backlight_level(BACKLIGHT_MAX);
	lcd_puts(0, 6, "YAPP");
	lcd_puts(1, 1, "ANCHOR WATCHER");

	CCPR4L=0x3e;                // set the beeper duty cycle to 50% 
	__delay_ms(150);
	CCPR4L=0x00;                // set the beeper duty cycle to 0% 
	
	__delay_ms(650);
	set_backlight_level(backlight);
	
	show_waiting_for_data_display();
		
	while(1)
	{
		clear_watchdog();

		// process waiting seatalk messages
		parse_next_seatalk_message();
		
		switch(app_state)
		{
			case WAITING_FOR_DATA:
				do_waiting_for_data_state();
				break;
				
			case STANDBY:
				do_standby_state();
				break;
				
			case WATCHING:
				do_watching_state();
				break;

			case ALARM:
				do_alarm_state();
				break;

			case SETUP:
				do_setup_state();
				break;
		}
	}
}

static void do_waiting_for_data_state(void)
{
	if(buttons==TOP_BUTTON)
	{
		enter_setup_state();
		buttons=0;
	}
	else if(tick)
	{
		tick=FALSE;
		if(check_data_availability()==0)
		{
			app_state=STANDBY;
			show_standby_display();
		}
	}			
}	

static void show_waiting_for_data_display(void)
{
	lcd_clear();
	lcd_puts(0, 0, "NO DATA    SETUP");
}	

static void do_standby_state(void)
{
	if(buttons>0)
	{
		if(buttons==TOP_BUTTON)
		{
			enter_setup_state();
		}
		else if(buttons==BOTTOM_BUTTON && (depth_enabled || distance_enabled || sog_enabled || wind_enabled || heading_enabled))
		{
			earliest_next_alarm_time=0UL;
			app_state=WATCHING;
			show_watching_display();
			wind_alarm_count=0;
			depth_alarm_count=0;
			sog_alarm_count=0;
			distance_alarm_count=0;
			heading_alarm_count=0;
			if(distance_enabled)
			{
				latitude_dfd_ref=(float)latitude_degrees;
				if(latitude_degrees>=0.0f)
				{
					latitude_dfd_ref+=latitude_minutes/60.0f;
				}
				else
				{
					latitude_dfd_ref-=latitude_minutes/60.0f;
				}	
	
				longitude_dfd_ref=(float)longitude_degrees;		
				if(longitude_degrees>0.0f)
				{
					longitude_dfd_ref+=longitude_minutes/60.0f;
				}
				else
				{
					longitude_dfd_ref-=longitude_minutes/60.0f;
				}
			}

			// if heading available take the reference point as current heading, else immediate no data alarm
			if(heading_enabled)
			{
				heading_ref=heading;
			}
		}	
		buttons=0;
	}
	else if(tick)
	{
		tick=FALSE;
		if(check_data_availability()>0)
		{
			app_state=WAITING_FOR_DATA;
			show_waiting_for_data_display();
		}		
	}
}
	
static void show_standby_display(void)
{
	lcd_clear();
	lcd_puts(0, 0, "STANDBY    SETUP");
	if(depth_enabled || distance_enabled || sog_enabled || wind_enabled || heading_enabled)
	{
		lcd_puts(1, 0, "           START");
	}	
}	

static void do_watching_state(void)
{
	if(buttons==TOP_BUTTON)
	{
		app_state=STANDBY;
		show_standby_display();
		buttons=0;
	}		
	else if(tick)
	{
		tick=FALSE;
		if(0x01&(unsigned char)one_second_tick_count)		
		{
			show_watching_display();
		}	
		
		if(one_second_tick_count>earliest_next_alarm_time)
		{
			if(distance_enabled)
			{
				// check distance from set point
				latitude_dfd=(float)latitude_degrees;
		
				if(latitude_degrees>=0.0f)
				{
					latitude_dfd+=latitude_minutes/60.0f;
				}
				else
				{
					latitude_dfd-=latitude_minutes/60.0f;
				}	
		
				longitude_dfd=(float)longitude_degrees;		
				if(longitude_degrees>0.0f)
				{
					longitude_dfd+=longitude_minutes/60.0f;
				}
				else
				{
					longitude_dfd-=longitude_minutes/60.0f;
				}
		
				distance_m=distance_between_points(latitude_dfd, longitude_dfd, latitude_dfd_ref, longitude_dfd_ref);
				if(distance_m>max_distance && distance_m<1000.0f)
				{
					distance_alarm_count++;
					if(distance_alarm_count>9)
					{
						alarm_type=alarm_distance;
					}
				}
				else
				{
					distance_alarm_count=0;
				}
			}
	
			if(depth_enabled)
			{
				// check depth
				if(depth<(((float)min_depth)/10.0f))   
				{
					depth_alarm_count++;
					if(depth_alarm_count>4)
					{
						alarm_type=alarm_depth;
					}
				}
				else
				{
					depth_alarm_count=0;
				}
			}
	
			if(sog_enabled)
			{
				// check speed over ground
				if(sog>((float)max_sog)/10.0f)
				{
					sog_alarm_count++;
					if(sog_alarm_count>4)
					{
						alarm_type=alarm_sog;
					}
				}
				else
				{
					sog_alarm_count=0;
				}
			}
	
			if(wind_enabled)
			{
				if((unsigned char)aws>max_wind)
				{
					wind_alarm_count++;
					if(wind_alarm_count>2)
					{
						alarm_type=alarm_wind;
					}
				}
				else
				{
					wind_alarm_count=0;
				}
			}
			
			if(heading_enabled)
			{
				// check heading difference from set point
				heading_difference=get_heading_error((int)heading, (int)heading_ref);
				if(heading_difference>max_heading && heading_difference<=180)
				{
					heading_alarm_count++;
					if(heading_alarm_count>4)
					{
						alarm_type=alarm_heading;
					}
				}
				else
				{
					heading_alarm_count=0;
				}
			}	
			
			// no data alarm situation cannot override an anchor alarm
			if(alarm_type==alarm_none && check_data_availability()>0)
			{
				alarm_type=alarm_no_data;
			}
			
			if(alarm_type!=alarm_none)
			{
				app_state=ALARM;
				show_alarm_display();
				CCPR4L=0x3e;         	// set the beeper duty cycle to 50% 
			}	
		}
	}	
}

static void show_watching_display(void)
{
	static unsigned char next_data=0;
	
	lcd_clear();
	if(one_second_tick_count>earliest_next_alarm_time)
	{
		lcd_puts(0, 0, "WATCHING    STOP");	
	}
	else
	{
		lcd_puts(0, 0, "RE-ARMING   STOP");	
	}		
	
	if(next_data==0)
	{
		if(!depth_enabled)
		{
			strcpypgm2ram(text_line, "DEPTH OFF");
		}
		else
		{
			sprintf(text_line, "DEPTH %u.%u m",  (unsigned int)depth, (unsigned int)(frac(depth)*10.f));
		}
	}	
	else if(next_data==1)
	{
		if(!sog_enabled)
		{
			strcpypgm2ram(text_line, "SOG OFF");
		}
		else
		{
			sprintf(text_line, "SOG %u.%u Kts",  (unsigned int)sog, (unsigned int)(frac(sog)*10.f));
		}
	}	
	else if(next_data==2)
	{
		if(!wind_enabled)
		{
			strcpypgm2ram(text_line, "WINDSPEED OFF");
		}
		else
		{
			sprintf(text_line, "WINDSPEED %u Kts",  (unsigned int)aws);
		}
	}	
	else if(next_data==3)
	{
		if(!heading_enabled)
		{
			strcpypgm2ram(text_line, "HEADING \x02 OFF");
		}
		else
		{
			sprintf(text_line, "HEADING \x02 %u \x01",  get_heading_error((int)heading, (int)heading_ref));
		}
	}	
	else if(next_data==4)
	{
		if(!distance_enabled)
		{
			strcpypgm2ram(text_line, "POSITION \x03 OFF");
		}
		else
		{
			if(((unsigned int)distance_m)<1000)	
			{
				sprintf(text_line, "POSITION \x03 %u m",  (unsigned int)distance_m);
			}
			else
			{
				strcatpgm2ram(text_line, "POSITION \x03 > 1km");
			}	
		}
	}	
	
	next_data++;
	if(next_data==5)
	{
		next_data=0;
	}
		
	lcd_puts_ram(1, 0, text_line);
}		

static void do_alarm_state(void)
{
	static int alternate=0;
	
	if(alarm_type!=alarm_none)
	{
		pulse_backlight();
	}	
		
	if(buttons>0)
	{
		if(buttons==TOP_BUTTON)
		{
			alarm_type=alarm_none;
			set_backlight_level(backlight);
			if(rearm_time==0)
			{
				earliest_next_alarm_time=one_second_tick_count+5UL;
			}
			else
			{	
				earliest_next_alarm_time=one_second_tick_count+((unsigned long)rearm_time)*60UL;
			}	
			if(check_data_availability()==0)
			{
				app_state=WATCHING;
				show_watching_display();
			}
			else
			{
				app_state=WAITING_FOR_DATA;
				show_waiting_for_data_display();
			}		
			CCPR4L=0x00;                // set the beeper duty cycle to 0%
		}	
		buttons=0;
	}
	else if(alarm_type==alarm_no_data)
	{
		if(check_data_availability()==0)
		{
			// this alarm self cancels if alarm situation clears
			alarm_type=alarm_none;
			CCPR4L=0x00;                // set the beeper duty cycle to 0% 
			app_state=WATCHING;
			show_watching_display();
		}	
		else if(tick)
		{
			alternate=!alternate;
			if(alternate)
			{
				CCPR4L=0x3e;                // set the beeper duty cycle to 50% 				
			}
			else
			{
				CCPR4L=0x00;                // set the beeper duty cycle to 0% 
			}		
			tick=FALSE;
		}	
	}
}
	
static void show_alarm_display(void)
{
	lcd_clear();
	
	text_line[0]='\0';
	
	switch(alarm_type)
	{
		case alarm_no_data:
			lcd_puts(0, 0, "DATA ALARM    OK");
			break;
			
		case alarm_depth:
			lcd_puts(0, 0, "DEPTH ALARM   OK");
			sprintf(text_line, "DEPTH %u.%u m",  (unsigned int)depth, (unsigned int)(frac(depth)*10.f));
			break;			
			
		case alarm_sog:
			lcd_puts(0, 0, "SOG ALARM     OK");
			sprintf(text_line, "SOG %u.%u Kts",  (unsigned int)sog, (unsigned int)(frac(sog)*10.f));
			break;	
			
		case alarm_wind:
			lcd_puts(0, 0, "WIND ALARM    OK");
			sprintf(text_line, "WINDSPEED %u Kts",  (unsigned int)aws);
			break;		
			
		case alarm_distance:	
			lcd_puts(0, 0, "DIST ALARM    OK");
			sprintf(text_line, "DISTANCE %u m",  (unsigned int)distance_m);
			break;	
			
		case alarm_heading:
			lcd_puts(0, 0, "HEADING ALARM OK");
			sprintf(text_line, "HEADING \x02 %u\x01",  get_heading_error((int)heading, (int)heading_ref));
			break;
	}
	
	lcd_puts_ram(1, 0, text_line);
}	

static void do_setup_state(void)
{
	if(buttons>0)
	{	
		if(setup_state==SETUP_STATE_MAIN)
		{
			if(buttons==TOP_BUTTON)
			{
				main_menu_position++;
				if(main_menu_position==MAIN_MENU_COUNT)
				{

					CCPR4L=0x00;                		// set the beeper duty cycle to 0% 
					set_backlight_level(backlight);		// backlight back to normal
					app_state=WAITING_FOR_DATA;
					show_waiting_for_data_display();
					buttons=0;						
					return;
				}
				show_setup_display();
			}
			else if(buttons==BOTTOM_BUTTON)
			{					
				switch(main_menu_position)
				{
					case 0:
						setup_state=SETUP_STATE_DEPTH;		
						break;
	
					case 1:
						setup_state=SETUP_STATE_SOG;
						break;

					case 2:
						setup_state=SETUP_STATE_WIND_SPEED;
						break;
						
					case 3:
						setup_state=SETUP_STATE_HEADING;
						break;
						
					case 4:
						setup_state=SETUP_STATE_POSITION;
						break;
						
					case 5:
						setup_state=SETUP_STATE_BACKLIGHT;
						break;
						
					case 6:
						setup_state=SETUP_STATE_REARM_TIME;
						break;
						
					case 7:
						setup_state=SETUP_STATE_ALARM_TEST;	
						CCPR4L=0x3e;                // set the beeper duty cycle to 50% 						
						set_backlight_level(BACKLIGHT_MAX);
						break;
				}
			}
		}
		else if(setup_state==SETUP_STATE_DEPTH)
		{
			if(buttons==TOP_BUTTON)
			{
				if(depth_enabled)
				{
					setup_state=SETUP_STATE_DEPTH_VALUE;
				}
				else
				{
					setup_state=SETUP_STATE_MAIN;
					main_menu_position++;
				}		
			}	
			else if(buttons==BOTTOM_BUTTON)
			{
				depth_enabled=!depth_enabled;
				int_EEPROM_putc(9, depth_enabled);
			}	
		}	
		else if(setup_state==SETUP_STATE_DEPTH_VALUE)
		{
			if(buttons==TOP_BUTTON)
			{
				setup_state=SETUP_STATE_MAIN;
				main_menu_position++;		
			}			
			else if(buttons==BOTTOM_BUTTON)
			{
				min_depth+=1;
				if(min_depth>250)
				{
					min_depth=0;
				}	
				int_EEPROM_putc(7, min_depth>>8);	
				int_EEPROM_putc(8, min_depth&0xff);	
			}	
			else if(buttons==BOTTOM_BUTTON_LONG)
			{
				min_depth+=10;
				if(min_depth>250)
				{
					min_depth=0;
				}	
				int_EEPROM_putc(7, min_depth>>8);	
				int_EEPROM_putc(8, min_depth&0xff);	
			}		
		}
		else if(setup_state==SETUP_STATE_SOG)
		{
			if(buttons==TOP_BUTTON)
			{
				if(sog_enabled)
				{
					setup_state=SETUP_STATE_SOG_VALUE;
				}
				else
				{
					setup_state=SETUP_STATE_MAIN;
					main_menu_position++;
				}		
			}			
			else if(buttons==BOTTOM_BUTTON)
			{
				sog_enabled=!sog_enabled;
				int_EEPROM_putc(13, sog_enabled);
			}		
		}	
		else if(setup_state==SETUP_STATE_SOG_VALUE)
		{
			if(buttons==TOP_BUTTON)
			{
				setup_state=SETUP_STATE_MAIN;
				main_menu_position++;		
			}			
			else if(buttons==BOTTOM_BUTTON)
			{
				max_sog+=1;
				if(max_sog>60)
				{
					max_sog=0;
				}	
				int_EEPROM_putc(12, max_sog);	
			}	
			else if(buttons==BOTTOM_BUTTON_LONG)
			{
				max_sog+=10;
				if(max_sog>60)
				{
					max_sog=0;
				}	
				int_EEPROM_putc(12, max_sog);	
			}		
		}		
		else if(setup_state==SETUP_STATE_WIND_SPEED)
		{
			if(buttons==TOP_BUTTON)
			{
				if(wind_enabled)
				{
					setup_state=SETUP_STATE_WIND_SPEED_VALUE;
				}
				else
				{
					setup_state=SETUP_STATE_MAIN;
					main_menu_position++;
				}		
			}			
			else if(buttons==BOTTOM_BUTTON)
			{
				wind_enabled=!wind_enabled;
				int_EEPROM_putc(11, wind_enabled);
			}		
		}	
		else if(setup_state==SETUP_STATE_WIND_SPEED_VALUE)
		{
			if(buttons==TOP_BUTTON)
			{
				setup_state=SETUP_STATE_MAIN;
				main_menu_position++;		
			}			
			else if(buttons==BOTTOM_BUTTON)
			{
				max_wind+=1;
				if(max_wind>50)
				{
					max_wind=0;
				}	
				int_EEPROM_putc(10, max_wind);	
			}	
			else if(buttons==BOTTOM_BUTTON_LONG)
			{
				max_wind+=10;
				if(max_wind>50)
				{
					max_wind=0;
				}	
				int_EEPROM_putc(10, max_wind);	
			}		
		}		
		else if(setup_state==SETUP_STATE_HEADING)
		{
			if(buttons==TOP_BUTTON)
			{
				if(heading_enabled)
				{
					setup_state=SETUP_STATE_HEADING_VALUE;
				}
				else
				{
					setup_state=SETUP_STATE_MAIN;
					main_menu_position++;
				}		
			}			
			else if(buttons==BOTTOM_BUTTON)
			{
				heading_enabled=!heading_enabled;
				int_EEPROM_putc(15, heading_enabled);
			}		
		}	
		else if(setup_state==SETUP_STATE_HEADING_VALUE)
		{
			if(buttons==TOP_BUTTON)
			{
				setup_state=SETUP_STATE_MAIN;
				main_menu_position++;		
			}			
			else if(buttons==BOTTOM_BUTTON)
			{
				max_heading+=1;
				if(max_heading>179)
				{
					max_heading=0;
				}	
				int_EEPROM_putc(14, max_heading);	
			}	
			else if(buttons==BOTTOM_BUTTON_LONG)
			{
				max_heading+=10;
				if(max_heading>179)
				{
					max_heading=0;
				}	
				int_EEPROM_putc(12, max_heading);	
			}		
		}
		else if(setup_state==SETUP_STATE_POSITION)
		{
			if(buttons==TOP_BUTTON)
			{
				if(distance_enabled)
				{
					setup_state=SETUP_STATE_POSITION_VALUE;
				}
				else
				{
					setup_state=SETUP_STATE_MAIN;
					main_menu_position++;
				}		
			}			
			else if(buttons==BOTTOM_BUTTON)
			{
				distance_enabled=!distance_enabled;
				int_EEPROM_putc(6, distance_enabled);
			}		
		}	
		else if(setup_state==SETUP_STATE_POSITION_VALUE)
		{
			if(buttons==TOP_BUTTON)
			{
				setup_state=SETUP_STATE_MAIN;
				main_menu_position++;		
			}				
			else if(buttons==BOTTOM_BUTTON)
			{
				max_distance+=1;
				if(max_distance>250)
				{
					max_distance=10;
				}	
				int_EEPROM_putc(5, max_distance);	
			}	
			else if(buttons==BOTTOM_BUTTON_LONG)
			{
				max_distance+=10;
				if(max_distance>250)
				{
					max_distance=10;
				}	
				int_EEPROM_putc(5, max_distance);	
			}	
		}	
		else if(setup_state==SETUP_STATE_BACKLIGHT)
		{
			if(buttons==TOP_BUTTON)
			{
				int_EEPROM_putc(17, backlight);
				setup_state=SETUP_STATE_MAIN;
				main_menu_position++;		
			}			
			else if(buttons==BOTTOM_BUTTON)
			{
				backlight++;
				if(backlight>BACKLIGHT_MAX)
				{
					backlight=0;
				}
				set_backlight_level(backlight);
			}
		}	
		else if(setup_state==SETUP_STATE_REARM_TIME)
		{
			if(buttons==TOP_BUTTON)
			{
				int_EEPROM_putc(16, rearm_time);
				setup_state=SETUP_STATE_MAIN;
				main_menu_position++;		
			}
			else if(buttons==BOTTOM_BUTTON)
			{
				rearm_time++;
				if(rearm_time>REARM_TIME_MAX)
				{
					rearm_time=0;
				}
			}
			else if(buttons==BOTTOM_BUTTON_LONG)
			{
				rearm_time+=10;
				if(rearm_time>REARM_TIME_MAX)
				{
					rearm_time=0;
				}
			}
		}	
		else if(setup_state==SETUP_STATE_ALARM_TEST)
		{
			if(buttons==TOP_BUTTON)
			{
				show_standby_display();
				CCPR4L=0x00;                // set the beeper duty cycle to 0% 
				app_state=STANDBY;	
				set_backlight_level(backlight);	
				buttons=0;
				return;
			}	
		}	
		
		buttons=0;	  
		show_setup_display();
	}
}
	
static void show_setup_display(void)
{
	lcd_clear();
	
	switch(setup_state)
	{
		case SETUP_STATE_MAIN:
			lcd_puts(0, 0, main_menu_items[main_menu_position]);
			lcd_puts(1, 10, "SELECT");
			if(main_menu_position==MAIN_MENU_COUNT-1)
			{
				lcd_puts(0, 12, "EXIT");
			}	
			else
			{
				lcd_puts(0, 12, "NEXT");
			}	

			switch(main_menu_position)
			{
				case 0:		// depth
					if(depth_enabled)
					{
						strcpypgm2ram(text_line, "ON");
					}
					else
					{
						strcpypgm2ram(text_line, "OFF");
					}		
					break;
					
				case 1:		// sog
					if(sog_enabled)
					{
						strcpypgm2ram(text_line, "ON");
					}
					else
					{
						strcpypgm2ram(text_line, "OFF");
					}		
					break;
					
				case 2:		// windspeed
					if(wind_enabled)
					{
						strcpypgm2ram(text_line, "ON");
					}
					else
					{
						strcpypgm2ram(text_line, "OFF");
					}		
					break;
					
				case 3:		// heading
					if(heading_enabled)
					{
						strcpypgm2ram(text_line, "ON");
					}
					else
					{
						strcpypgm2ram(text_line, "OFF");
					}		
					break;
					
				case 4:		// position
					if(distance_enabled)
					{
						strcpypgm2ram(text_line, "ON");
					}
					else
					{
						strcpypgm2ram(text_line, "OFF");
					}		
					break;
					
				case 5:		// backlight
					sprintf(text_line, "%u", backlight); 		
					break;
					
				case 6:		// rearm time
					sprintf(text_line, "%um", rearm_time); 		
					break;
					
				default:
					text_line[0]='\0';
					break;
			}	
			lcd_puts_ram(1, 0, text_line);
			break;
			
		case SETUP_STATE_DEPTH:
			if(depth_enabled)
			{
				lcd_puts(0, 0, "ON");
			}
			else
			{
				lcd_puts(0, 0, "OFF");
			}		
			lcd_puts(0, 12, "NEXT");
			lcd_puts(1, 10, "TOGGLE");
			break;
			
		case SETUP_STATE_DEPTH_VALUE:
			sprintf(text_line, "%u.%u m", min_depth/10, min_depth%10); 
			lcd_puts_ram(0, 0, text_line);	
			lcd_puts(0, 12, "NEXT");
			lcd_puts(1, 12, "INCR");
			break;
			
		case SETUP_STATE_SOG:
			if(sog_enabled)
			{
				lcd_puts(0, 0, "ON");
			}
			else
			{
				lcd_puts(0, 0, "OFF");
			}		
			lcd_puts(0, 12, "NEXT");
			lcd_puts(1, 10, "TOGGLE");
			break;
			
		case SETUP_STATE_SOG_VALUE:
			sprintf(text_line, "%u.%u Kts", max_sog/10, max_sog%10); 
			lcd_puts_ram(0, 0, text_line);	
			lcd_puts(0, 12, "NEXT");
			lcd_puts(1, 12, "INCR");
			break;
			
		case SETUP_STATE_WIND_SPEED:
			if(wind_enabled)
			{
				lcd_puts(0, 0, "ON");
			}
			else
			{
				lcd_puts(0, 0, "OFF");
			}		
			lcd_puts(0, 12, "NEXT");
			lcd_puts(1, 10, "TOGGLE");
			break;
			
		case SETUP_STATE_WIND_SPEED_VALUE:
			sprintf(text_line, "%u Kts", max_wind); 
			lcd_puts_ram(0, 0, text_line);	
			lcd_puts(0, 12, "NEXT");
			lcd_puts(1, 12, "INCR");
			break;
			
		case SETUP_STATE_HEADING:
			if(heading_enabled)
			{
				lcd_puts(0, 0, "ON");
			}
			else
			{
				lcd_puts(0, 0, "OFF");
			}		
			lcd_puts(0, 12, "NEXT");
			lcd_puts(1, 10, "TOGGLE");
			break;
			
		case SETUP_STATE_HEADING_VALUE:
			sprintf(text_line, "%u \x01", max_heading); 
			lcd_puts_ram(0, 0, text_line);	
			lcd_puts(0, 12, "NEXT");
			lcd_puts(1, 12, "INCR");
			break;
			
		case SETUP_STATE_POSITION:
			if(distance_enabled)
			{
				lcd_puts(0, 0, "ON");
			}
			else
			{
				lcd_puts(0, 0, "OFF");
			}		
			lcd_puts(0, 12, "NEXT");
			lcd_puts(1, 10, "TOGGLE");
			break;
			
		case SETUP_STATE_POSITION_VALUE:
			sprintf(text_line, "%u m", max_distance); 
			lcd_puts_ram(0, 0, text_line);	
			lcd_puts(0, 12, "NEXT");
			lcd_puts(1, 12, "INCR");
			break;
			
		case SETUP_STATE_BACKLIGHT:
			strcpypgm2ram(text_line, "LEVEL=");
			itoa(backlight, number_buffer);
			strcat(text_line, number_buffer);
			lcd_puts_ram(0, 0, text_line);
			lcd_puts(0, 12, "NEXT");
			lcd_puts(1, 12, "INCR");

			break;
			
		case SETUP_STATE_REARM_TIME:
			strcpypgm2ram(text_line, "TIME=");
			itoa(rearm_time, number_buffer);
			strcat(text_line, number_buffer);
			strcatpgm2ram(text_line, "m");
			lcd_puts_ram(0, 0, text_line);
			lcd_puts(0, 12, "NEXT");
			lcd_puts(1, 12, "INCR");
			break;
			
		case SETUP_STATE_ALARM_TEST:
			lcd_puts(0, 0, "ALARM TEST  EXIT");
			lcd_puts(1, 0, SOFTWARE_VERSION_NUMBE);
			break;
	}		
}

static void enter_setup_state(void)
{
	app_state=SETUP;
	setup_state=SETUP_STATE_MAIN;
	main_menu_position=0;
	show_setup_display();
}		

static void clear_watchdog(void)
{
	ClrWdt();
}	

static void seatalk_message_handler(unsigned char message_type)
{		
	switch(message_type)
	{
		case DEPTH:
			depth_received_time=one_second_tick_count;
			break;

		case SOG:
			sog_received_time=one_second_tick_count;
			break;
			
		case LATITUDE:
			lat_received_time=one_second_tick_count;
			break;
			
		case LONGITUDE:
			long_received_time=one_second_tick_count;
			break;
			
		case APPARENT_WIND_ANGLE:
			awa_received_time=one_second_tick_count;
			break;
			
		case APPARENT_WIND_SPEED:
			aws_received_time=one_second_tick_count;
			break;

		case HEADING2:
		case HEADING1:
			heading_received_time=one_second_tick_count;
			break;		
	}
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
	lcd_set_cg_character(2, turn_character);
	lcd_set_cg_character(3, move_character);
	
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
	TRISBbits.TRISB0=0;		      
	CCPTMRS1bits.C4TSEL=1;		// timer 4
	CCPR4L=0x00;                // set the beeper duty cycle to 50%     
    CCP4CONbits.DC4B=0;       	// the 2 lsb of the duty cycle, section 15.4.2, register 15-1
    CCP4CONbits.CCP4M=0xf;      // set CCP4 to PWM mode, register 15-1

    // setup capture/compare 3, this is used for the backlight pwm output
    TRISCbits.TRISC6=0;  		
	CCPTMRS0bits.C3TSEL=1;		// timer 4
    CCPR3L=0x88;                // set the beeper duty cycle to 0% 
    CCP3CONbits.DC3B=0;       	// the 2 lsb of the duty cycle, section 15.4.2, register 15-1
    CCP3CONbits.CCP3M=0xf;      // set CCP3 to PWM mode, register 15-1

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
	
	// init the eeprom
	if(int_EEPROM_getc(0)!='Y' || int_EEPROM_getc(1)!='A' || int_EEPROM_getc(2)!='P' || int_EEPROM_getc(3)!='P')
	{
        // nothing or corrupt data in the eeprom so initialize it
		int_EEPROM_putc(4, 0);		// version
		int_EEPROM_putc(5, max_distance);	
		int_EEPROM_putc(6, distance_enabled);
		int_EEPROM_putc(7, min_depth>>8);	
		int_EEPROM_putc(8, min_depth&0xff);	
		int_EEPROM_putc(9, depth_enabled);
		int_EEPROM_putc(10, max_wind);
		int_EEPROM_putc(11, wind_enabled);
		int_EEPROM_putc(12, max_sog);
		int_EEPROM_putc(13, sog_enabled);
		int_EEPROM_putc(14, max_heading);
		int_EEPROM_putc(15, heading_enabled);
		int_EEPROM_putc(16, rearm_time);
		int_EEPROM_putc(17, BACKLIGHT_MAX);
		int_EEPROM_putc(0, 'Y');
		int_EEPROM_putc(1, 'A');
		int_EEPROM_putc(2, 'P');
		int_EEPROM_putc(3, 'P');
	}
	else
	{
        // eeprom marker ok so read settings
		max_distance=int_EEPROM_getc(5);
		distance_enabled=int_EEPROM_getc(6);
		min_depth=((unsigned int)(int_EEPROM_getc(7)))<<8;
		min_depth+=int_EEPROM_getc(8);
		depth_enabled=int_EEPROM_getc(9);
		max_wind=int_EEPROM_getc(10);
		wind_enabled=int_EEPROM_getc(11);
		max_sog=int_EEPROM_getc(12);
		sog_enabled=int_EEPROM_getc(13);
		max_heading=int_EEPROM_getc(14);
		heading_enabled=int_EEPROM_getc(15);
		rearm_time=int_EEPROM_getc(16);
		backlight=int_EEPROM_getc(17);	
	}
 
	// turn on interrupts
	RCONbits.IPEN=0;	        // disable interrupt priority, section 9.2
    PIE5bits.TMR6IE=1;          // enable timer6 interrupt 
	INTCONbits.RBIE=1;			// enable port b change interrupts
	INTCONbits.PEIE=1;          // enable all unmasked peripheral interrupts
	INTCONbits.GIE=1;           // globally enable interrupts
} 

static unsigned char check_data_availability(void)
{
	unsigned char alarm_needed=0;

	if(distance_enabled)
	{
		// check lat and long is being received
		if(one_second_tick_count-lat_received_time>MAX_DATA_AGE || one_second_tick_count-long_received_time>MAX_DATA_AGE)
		{
			alarm_needed|=0b00000001;
		}
		else
		{
			alarm_needed&=0b11111110;
		}
	}
	else
	{
		alarm_needed&=0b11111110;
	}

	if(depth_enabled)
	{
		// check depth is being received
		if(one_second_tick_count-depth_received_time>MAX_DATA_AGE)
		{
			alarm_needed|=0b00000010;
		}
		else
		{
			alarm_needed&=0b11111101;
		}
	}
	else
	{
		alarm_needed&=0b11111101;
	}

	if(sog_enabled)
	{
		// check speed over ground is being received
		if(one_second_tick_count-sog_received_time>MAX_DATA_AGE)
		{
			alarm_needed|=0b00000100;
		}
		else
		{
			alarm_needed&=0b11111011;
		}
	}
	else
	{
		alarm_needed&=0b11111011;
	}

	if(wind_enabled)
	{
		// check wind speed is being received
		if(one_second_tick_count-awa_received_time>MAX_DATA_AGE || one_second_tick_count-aws_received_time>MAX_DATA_AGE)
		{
			alarm_needed|=0b00001000;
		}
		else
		{
			alarm_needed&=0b11110111;
		}
	}
	else
	{
		alarm_needed&=0b11110111;
	}

	if(heading_enabled)
	{
		// check heading is being received
		if(one_second_tick_count-heading_received_time>MAX_DATA_AGE)
		{
			alarm_needed|=0b00010000;
		}
		else
		{
			alarm_needed&=0b11101111;
		}
	}
	else
	{
		alarm_needed&=0b11101111;
	}

	return alarm_needed;
}

static int abs(int x)
{
	return x<0 ? -x : x;
}

static int get_heading_error(int initial, int final)
{
	int clock_wise=final-initial;
	int counter_clock_wise=360-final+initial;
	return abs((abs(clock_wise)<=abs(counter_clock_wise)) ? clock_wise : -counter_clock_wise);
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
		if(c==12)					
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
		else if(c>12 && c<60)
		{
			if(!PORTBbits.RB5 && !PORTBbits.RB4)
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
