#ifndef _GPS_REPEATER_H
#define _GPS_REPEATER_H

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

#define NMEA_MESSAGE_MAX_LENGTH		82
#define NUMBER_NMEA_MESSAGES		5

enum message_state_t
{
	MS_READY,
	MS_READING,
	MS_DONE,
};

typedef enum
{
	LATITUDE,
	LONGITUDE
} coord_t;

void init_app(void);
void update_display(void);


#endif