#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "george_corrector.h"
#include "seatalk.h"
#include "hd44780_16x4_lcd.h"

extern char string_buffer[50];
extern float depth;
extern float boatspeed;
extern float trip;
extern float boat_log;
extern signed char variation;
extern float tws;
extern float twa;
extern float awa;
extern float aws;
extern int heading;
extern int heading_true;
extern float sog;
extern unsigned int cog;
extern float temperature;
extern volatile unsigned long tick_count;
extern unsigned long ttg;
extern unsigned char autopilot_command;
extern int auto_course;

void secondstoa(long seconds, char *text);

void update_display(void)
{
	char text_line[17];

	sprintf(text_line, "TIME %ld", tick_count);
	lcd_clear_line(1);
	lcd_puts_ram(1, text_line);

	if(autopilot_command==APC_STANDBY)
	{
		sprintf(text_line, "STANDBY");
		lcd_clear_line(3);
		lcd_puts_ram(3, text_line);
	}
	else if(autopilot_command==APC_AUTO)
	{
		sprintf(text_line, "AUTO");
		lcd_clear_line(3);
		lcd_puts_ram(3, text_line);
	}

	sprintf(text_line, "AUTOCOURSE %d", auto_course);
	lcd_clear_line(2);
	lcd_puts_ram(2, text_line);
}	

/** Convert a relative time value in seconds to seconds, minutes&seconds or hours&minutes&seconds in text form. */
void secondstoa(long seconds, char *text)
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

