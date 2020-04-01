#ifndef _ANCHOR_WATCHER_2_H
#define _ANCHOR_WATCHER_2_H

#ifndef TRUE
#define TRUE 						1
#endif
#ifndef FALSE
#define FALSE					 	0
#endif

#define SOFTWARE_VERSION_NUMBE		"V1.0"
#define MAX_DATA_AGE				15
#define BACKLIGHT_MAX				3	
#define REARM_TIME_MAX				15
#define TOP_BUTTON					1
#define BOTTOM_BUTTON				2
#define TOP_BUTTON_LONG				5
#define BOTTOM_BUTTON_LONG			6
#define NMEA_MESSAGE_MAX_LENGTH		82
#define NUMBER_NMEA_MESSAGES		5

typedef enum
{
	LATITUDE_T,
	LONGITUDE_T
} coord_t;

typedef enum 
{
	WAITING_FOR_DATA,
	STANDBY,
	WATCHING,
	ALARM,
	SETUP
} app_state_t;

typedef enum 
{
	alarm_none,
	alarm_no_data,
	alarm_depth,
	alarm_sog,
	alarm_wind,
	alarm_distance,
	alarm_heading
} t_alarm;

typedef enum 
{
	SETUP_STATE_MAIN,
	SETUP_STATE_DEPTH,
	SETUP_STATE_DEPTH_VALUE,
	SETUP_STATE_SOG,
	SETUP_STATE_SOG_VALUE,
	SETUP_STATE_WIND_SPEED,
	SETUP_STATE_WIND_SPEED_VALUE,
	SETUP_STATE_HEADING,
	SETUP_STATE_HEADING_VALUE,
	SETUP_STATE_POSITION,
	SETUP_STATE_POSITION_VALUE,
	SETUP_STATE_BACKLIGHT,
	SETUP_STATE_REARM_TIME,
	SETUP_STATE_ALARM_TEST
} setup_state_t;

typedef enum
{
	LATITUDE,
	LONGITUDE
} coord_t;

#endif
