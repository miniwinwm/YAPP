#ifndef _NMEA_H
#define _NMEA_H

#include "types.h"

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE 1
#endif

#define NMEA_MAX_MESSAGE_LENGTH		82
#define NMEA_MESSAGE_STORAGE		(1+NMEA_MAX_MESSAGE_LENGTH+1)
#define NUMBER_NMEA_MESSAGES		5
#define NMEA_OUT_BUFFER_SIZE		300

typedef enum
{
	NMEA_DPT,
	NMEA_DBT,
	NMEA_VHW,
	NMEA_RSA,
	NMEA_HDM,
	NMEA_HDG,
	NMEA_HDT,
	NMEA_MTW,
	NMEA_VLW,
	NMEA_VWR,
	NMEA_VWT,
	NMEA_MWV,
	NMEA_RMC,
	NMEA_GLL,
	// nmea add more message types here
	
	NMEA_UNKNOWN
} nmea_message_type_t;

unsigned char nmea_test_checksum(char *message);
void nmea_queue_message_to_send(char *message);
void nmea_process_next_message(void);
unsigned char nmea_get_message_period_milliseconds(nmea_message_type_t message_type);
nmea_message_type_t nmea_identify_message_type(char *message);
char *nmea_get_name_from_type(nmea_message_type_t message_type);

// nmea add more message types here
void nmea_DPT_send(float depth);
void nmea_DBT_send(float depth);
void nmea_VHW_send(float heading_true, float heading_magnetic, float boatspeed);
void nmea_RSA_send(float rudder_angle);
void nmea_HDM_send(float heading_magnetic);
void nmea_HDG_send(float heading_magnetic, float variation);
void nmea_HDT_send(float heading_true);
void nmea_MTW_send(float temperature);
void nmea_VLW_send(float trip, float log);
void nmea_VWR_send(float apparent_wind_angle, float apparent_wind_speed);
void nmea_VWT_send(float apparent_wind_angle, float apparent_wind_speed, float boatspeed);
void nmea_MWV_send(float apparent_wind_angle, float apparent_wind_speed, float boatspeed);
void nmea_RMC_send(nav_data_t *nav_data);
void nmea_GLL_send(nav_data_t *nav_data);

#endif
