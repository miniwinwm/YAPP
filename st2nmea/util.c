#include <string.h>
#include <math.h>
#include "util.h"
#include "sscanf.h"
#include "nmea.h"
#include "seatalk.h"
#include "st2nmea.h"
#include "eeprom.h"

extern char next_message_field[NMEA_MAX_MESSAGE_LENGTH+1];
extern char nmea_message[NMEA_MAX_MESSAGE_LENGTH+1];
extern unsigned char seatalk_settings[];
extern unsigned char nmea_settings[];
extern const unsigned char seatalk_default_settings_size;
extern const unsigned char nmea_default_settings_size;

static void util_display_nmea_settings(void);
static void util_display_seatalk_settings(void);	

unsigned int util_abs(signed int i)
{
	if(i<0)
	{
		return (unsigned int)-i;
	}
	
	return (unsigned int)i;
}	

void util_calc_true_wind_from_apparent(float apparent_wind_speed, float apparent_wind_angle, float boatspeed, float *true_wind_speed, float *true_wind_angle)
{
	float x;
	float y;
	float z;
	float t;
	
	if(boatspeed<0.01f)
	{
		*true_wind_speed=apparent_wind_speed;
		*true_wind_angle=apparent_wind_angle;
	}
	else
	{
		x=boatspeed*sin(apparent_wind_angle/DEGREES_TO_RADIANS);
		y=boatspeed*cos(apparent_wind_angle/DEGREES_TO_RADIANS);
		z=apparent_wind_speed-y;
		*true_wind_speed=sqrt(z*z+x*x);
		if(*true_wind_speed==0.0f)
		{
			*true_wind_angle=0.0f;
		}
		else
		{
			t=(M_PI_2-apparent_wind_angle/DEGREES_TO_RADIANS)+acos(x/(*true_wind_speed));
			*true_wind_angle=M_PI-t;
		}
		*true_wind_angle*=DEGREES_TO_RADIANS;
		if(*true_wind_angle<0.0f)
		{
			*true_wind_angle+=360.0f;
		}
	}
}	

void util_calc_apparent_wind_from_true(float true_wind_speed, float true_wind_angle, float boatspeed, float *apparent_wind_speed, float *apparent_wind_angle)
{
	float a;
	float b;
	
	if(true_wind_angle<1.0f || true_wind_angle>359.0f || boatspeed<0.1f)
	{
		*apparent_wind_angle=true_wind_angle;
		*apparent_wind_speed=true_wind_speed+boatspeed;
		return;
	}	
	
	if(true_wind_speed<0.1f)
	{
		*apparent_wind_angle=0.0f;
		*apparent_wind_speed=boatspeed;
		return;
	}	
	
	if(true_wind_angle>179.0f || true_wind_angle<181.0f)
	{
		*apparent_wind_angle=true_wind_angle;
		*apparent_wind_speed=true_wind_speed-boatspeed;
		if(*apparent_wind_speed<0.0f)
		{
			*apparent_wind_speed=-*apparent_wind_speed;
			*apparent_wind_angle=0.0f;
		}
		return;	
	}	
		
	a=true_wind_speed*sin(true_wind_angle/DEGREES_TO_RADIANS);
	b=true_wind_speed*cos(true_wind_angle/DEGREES_TO_RADIANS);
	*apparent_wind_speed=sqrt(a*a+(b+boatspeed)*(b+boatspeed));
	*apparent_wind_angle=atan((b+boatspeed)/a);
	*apparent_wind_angle*=DEGREES_TO_RADIANS;
	if(*apparent_wind_angle<0.0f)
	{
		*apparent_wind_angle+=360.0f;
	}
}
	
float frac(float f)
{
	if(f>=0.0f)
	{
		return f-floor(f);
	}
	else
	{
		return f-ceil(f);
	}
}	

float util_calc_heading_true(float heading_magnetic, float variation)
{
	float heading_true;

	heading_true=heading_magnetic+variation;
	if(heading_true<0.0f)
	{
		heading_true+=360.0f;
	}
	else if(heading_true>=360.0f)
	{
		heading_true-=360.0f;
	}

	return heading_true;
}

unsigned char util_get_field(unsigned char field_number, char *message, char *message_field, char delimeter)
{
	unsigned char delimeter_count=0;
	unsigned char next_char=0;
	unsigned char message_length;
	unsigned char field_length=0;

	message_length=strlen(message);

	// find start of field
	while(1)
	{
	    if(delimeter_count==field_number)
	    {
	        break;
	    }

		next_char++;
		if(next_char==message_length)
		{
		    message_field[0]='\0';
		    return 0;
		}

		if(message[next_char]==delimeter)
		{
		    delimeter_count++;
		}
	}

	// move over delimeter
	if(next_char>0)
	{
		next_char++;
	}

	// find field length
	while(1)
	{
	    if(message[next_char+field_length]==delimeter || message[next_char+field_length]=='\0')
	    {
	        break;
	    }
	    field_length++;
	}

	strncpy(message_field, message+next_char, field_length);
	message_field[field_length]='\0';
	
	return field_length;
}

static void util_display_nmea_settings(void)
{
	unsigned char i;

	// display nmea settings
	strcpypgm2ram(nmea_message, "\r\nNMEA-0183 Settings\r\n");
	nmea_queue_message_to_send(nmea_message);

	for(i=0; i<nmea_default_settings_size; i++)
	{
		strcpy(nmea_message, nmea_get_name_from_type(i));
		strcatpgm2ram(nmea_message, " ");
		strcat(nmea_message, ftoa((float)(nmea_settings[i]), 0));
		strcatpgm2ram(nmea_message, "s\r\n");
		nmea_queue_message_to_send(nmea_message);
	}
}
	
static void util_display_seatalk_settings(void)
{
	unsigned char i;

	// display seatalk settings
	strcpypgm2ram(nmea_message, "\r\nSeatalk Settings\r\n");
	nmea_queue_message_to_send(nmea_message);

	for(i=0; i<seatalk_default_settings_size; i++)
	{
		strcpy(nmea_message, util_btoh(seatalk_get_identifier_from_type(i)));
		strcatpgm2ram(nmea_message, " ");
		strcat(nmea_message, ftoa((float)seatalk_settings[i], 0));
		strcatpgm2ram(nmea_message, "s\r\n");
		nmea_queue_message_to_send(nmea_message);
	}

	strcpypgm2ram(nmea_message, "\r\n");
	nmea_queue_message_to_send(nmea_message);
}

char *util_padded_uitoa(unsigned int i, signed char digits)
{
	static char result[7];
	signed char number_of_digits=strlen(ftoa(fabs((float)i), 0));
	
	result[0]='\0';	
	
	while(digits-number_of_digits>0)
	{
		strcatpgm2ram(result, "0");
		digits--;
	}	
	
	strcat(result, ftoa(fabs((float)i), 0));
	
	return result;
}	

char *util_btoh(unsigned char b)
{
	static char result[3];
	
	result[0]=b>>4;
	if(result[0]<10)
	{
		result[0]+='0';
	}	
	else
	{
		result[0]+=('A'-10);
	}
	result[1]=b&0x0f;
	if(result[1]<10)
	{
		result[1]+='0';
	}	
	else
	{
		result[1]+=('A'-10);
	}	
	result[2]='\0';
	
	return result;
}	

// #N,DPT,5
// #S,00,3
void util_decode_setting_message(char *setting_message)
{
	nmea_message_type_t nmea_message_type;
	seatalk_message_type_t seatalk_message_type;
	unsigned char seatalk_message_id;
	unsigned int period;
	unsigned char i;
	unsigned char length;

	if(setting_message[0]=='O')
	{
		for(i=0; i<seatalk_default_settings_size; i++)
		{
			seatalk_settings[i]=0;
			int_EEPROM_putc(SEATALK_SETTINGS_BASE+i, 0);
		}	
		for(i=0; i<nmea_default_settings_size; i++)
		{
			nmea_settings[i]=0;
			int_EEPROM_putc(NMEA_SETTINGS_BASE+i, 0);
		}
	}	
	else if(setting_message[0]=='S')
	{
		// seatalk setting
		length=util_get_field(1, setting_message, next_message_field, ',');

		if(length==1 || length==2)
		{
			sscanf(next_message_field, "%x", &seatalk_message_id);
			seatalk_message_type=seatalk_identify_message_type(seatalk_message_id);
			if(seatalk_message_type!=SEATALK_UNKNOWN)
			{
				// get period from next field
				if(util_get_field(2, setting_message, next_message_field, ',')>0)
				{
					sscanf(next_message_field, "%u", &period);
					if(period<256)
					{
						int_EEPROM_putc(SEATALK_SETTINGS_BASE+seatalk_message_type, (unsigned char)period);
						seatalk_settings[seatalk_message_type]=(unsigned char)period;
					}
				}
			}
		}
		else
		{
			util_display_seatalk_settings();
		}	
	}
	else if(setting_message[0]=='N')
	{
		// nmea setting
		if(util_get_field(1, setting_message, next_message_field, ',')==3)
		{
			nmea_message_type=nmea_identify_message_type(next_message_field);
			if(nmea_message_type!=NMEA_UNKNOWN)
			{
				// get period from next field
				if(util_get_field(2, setting_message, next_message_field, ',')>0)
				{
					sscanf(next_message_field, "%u", &period);
					if(period<256)
					{
						int_EEPROM_putc(NMEA_SETTINGS_BASE+nmea_message_type, (unsigned char)period);
						nmea_settings[nmea_message_type]=(unsigned char)period;
					}
				}
			}
		}
		else
		{
			util_display_nmea_settings();
		}
	}
}
