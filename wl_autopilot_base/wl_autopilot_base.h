#ifndef _CREW_WATCHER_H
#define _CREW_WATCHER_H

#ifndef TRUE
#define TRUE 						1
#endif
#ifndef FALSE
#define FALSE					 	0
#endif

#define SOFTWARE_VERSION_NUMBE		"V1.0"
#define CHANNELS 					4

#define NMEA_MESSAGE_MAX_LENGTH		82
#define NUMBER_NMEA_MESSAGES		5
#define GPS_TIMEOUT_SECONDS			10
#define ALARM_REARM_TIME_SECONDS	90
#define TOP_BUTTON					1
#define BOTTOM_BUTTON				2
#define ALARM_TIME_MIN				15
#define ALARM_TIME_MAX				60
#define BACKLIGHT_MAX				3

typedef enum
{
	NORMALLY_CLOSED,
	NORMALLY_OPEN
} switch_state_t;	

typedef enum
{
	LATITUDE_T,
	LONGITUDE_T
} coord_t;

typedef enum 
{
	MONITORING,
	ALARM,
	CONFIRM_CANCEL_ALARM,
	SETTINGS,
	POSITION
} app_state_t;

typedef enum 
{
	MAIN,
	ALARM_TIME,
	CHANNEL_SETUP,
	BACKLIGHT,
	ALARM_OUTPUT
} settings_state_t;

typedef enum
{
	LATITUDE,
	LONGITUDE
} coord_t;

#endif
