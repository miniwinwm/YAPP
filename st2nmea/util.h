#ifndef _UTIL_H
#define _UTIL_H

#define M_PI 					3.1416f
#define M_PI_2		 			1.5708f
#define DEGREES_TO_RADIANS 		57.259f

unsigned char util_get_field(unsigned char field_number, char *message, char *message_field, char delimeter);
void util_decode_setting_message(char *setting_message);
float util_calc_heading_true(float heading_magnetic, float variation);
float frac(float f);
unsigned int util_abs(signed int i);
void util_calc_apparent_wind_from_true(float true_wind_speed, float true_wind_angle, float boatspeed, float *apparent_wind_speed, float *apparent_wind_angle);
void util_calc_true_wind_from_apparent(float apparent_wind_speed, float apparent_wind_angle, float boatspeed, float *true_wind_speed, float *true_wind_angle);
char *util_btoh(unsigned char b);
char *util_padded_uitoa(unsigned int i, signed char p);

#endif
