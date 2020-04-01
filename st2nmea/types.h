#ifndef _TYPES_H
#define _TYPES_H

typedef struct
{
	unsigned char year;
	unsigned char month;
	unsigned char date;
} date_t;	

typedef struct
{
	unsigned char hour;
	unsigned char minute;
 	unsigned char second;
} time_t;

typedef struct 
{
	time_t time;
	signed int latitude_degrees;
	float latitude_minutes;
	signed int longitude_degrees;
	float longitude_minutes;
	float sog;
	float cog;
	date_t date;
	float variation;
} nav_data_t;

#endif
