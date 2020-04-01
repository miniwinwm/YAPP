#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "seatalk_repeater_16x4.h"
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

void secondstoa(long seconds, char *text);

void update_display(void)
{
	char text_line[17];
	unsigned char scroll_position=tick_count%8;
	unsigned char i;
	unsigned char next_write_position=scroll_position;

#if defined SLIPSTREAM
	sprintf(text_line, "DEPTH %d.%d m", (int)depth, (int)((depth*10.0)-((unsigned int)depth)*10));
	lcd_clear_line(0);
	lcd_puts_ram(0, text_line);

	if(tws<99.9f)
	{

		sprintf(text_line, "TRUE WIND %d.%d K", (int)tws, (int)((tws*10.0f)-((unsigned int)tws)*10));
	}
	lcd_clear_line(1);
	lcd_puts_ram(1, text_line);

	if(twa>180.0f)
	{
		sprintf(text_line, "DIRECTION %03d P", (unsigned int)(360.0f-twa));
	}
	else
	{
		sprintf(text_line, "DIRECTION %03d S", (unsigned int)twa);
	}
	lcd_clear_line(2);
	lcd_puts_ram(2, text_line);

	sprintf(text_line, "HEADING %03d T", heading_true);
	lcd_clear_line(3);
	lcd_puts_ram(3, text_line);
#elif defined CUAN	
	sprintf(text_line, "DEPTH %d.%d m", (int)depth, (int)((depth*10.0)-((unsigned int)depth)*10));
	lcd_clear_line(0);
	lcd_puts_ram(0, text_line);

	sprintf(text_line, "BOATSPEED %d.%d", (int)boatspeed, (int)((boatspeed*10.0f)-((unsigned int)boatspeed)*10));
	lcd_clear_line(1);
	lcd_puts_ram(1, text_line);

	sprintf(text_line, "LOG %ld", (long)boat_log);
	lcd_clear_line(3);
	lcd_puts_ram(3, text_line);

	sprintf(text_line, "TRIP %d.%02d", (int)trip, (int)((trip*100.0f)-( (unsigned int)trip)*100));
	lcd_clear_line(2);
	lcd_puts_ram(2, text_line);
#else
	for(i=0; i<4; i++)
	{
		switch(next_write_position)
		{
			case 0:
				sprintf(text_line, "HEAD %03d M %03d T", heading, heading_true);
				break;

			case 1:
				if(tws<99.9f)
				{
					if(twa>180.0f)
					{
						sprintf(text_line, "T WIND %03dP %d.%d", (unsigned int)(360.0f-twa), (int)tws, (int)((tws*10.0f)-((unsigned int)tws)*10));
					}
					else
					{
						sprintf(text_line, "T WIND %03dS %d.%d", (unsigned int)twa, (int)tws, (int)((tws*10.0f)-((unsigned int)tws)*10));
					}
				}
				break;

			case 2:
				sprintf(text_line, "DEPTH %d.%dm", (int)depth, (int)((depth*10.0)-((unsigned int)depth)*10));
				break;

			case 3:
				if(boat_log<100000.0f && trip<1000.0f)
				{
					sprintf(text_line, "LOG %05d %03d.%02d", (int)boat_log, (int)trip, (int)((trip*100.0f)-( (unsigned int)trip)*100));
				}
				else
				{
					sprintf(text_line, "LOG %06d", (int)boat_log);
				}
				break;

			case 4:
				sprintf(text_line, "COG %03d SOG %d.%d", (unsigned int)(cog), (int)sog, (int)((sog*10.0f)-((unsigned int)sog)*10));
				break;

			case 5:
				if(aws<99.9f)
				{
					if(awa>180.0f)
					{
						sprintf(text_line, "A WIND %03dP %d.%d", (unsigned int)(360.0f-awa), (int)aws, (int)((aws*10.0f)-((unsigned int)aws)*10));
					}
					else
					{
						sprintf(text_line, "A WIND %03dS %d.%d", (unsigned int)awa, (int)aws, (int)((aws*10.0f)-((unsigned int)aws)*10));
					}
				}
				break;

			case 6:
				sprintf(text_line, "BOATSPEED %d.%d", (int)boatspeed, (int)((boatspeed*10.0f)-((unsigned int)boatspeed)*10));
				break;

			case 7:
#ifdef LAZY_KIPPER
				{
					char num_buf[16];
					secondstoa((long)ttg, num_buf);
					sprintf(text_line, "TTG %s", num_buf);
				}
#else
				if(temperature<0.0)
				{
					temperature=-temperature;
					sprintf(text_line, "TEMP -%d.%d C", (int)temperature, (int)((temperature*10.0f)-((unsigned int)temperature)*10));
				}
				else
				{
					sprintf(text_line, "TEMP %d.%d C", (int)temperature, (int)((temperature*10.0f)-((unsigned int)temperature)*10));
				}
#endif				
				break;
		}


		lcd_clear_line(i);
		lcd_puts_ram(i, text_line);

		next_write_position++;
		if(next_write_position==8)
		{
			next_write_position=0;
		}
	}
#endif
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

