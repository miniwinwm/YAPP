#ifndef _ST2NMEA_H
#define _ST2NMEA_H

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

enum message_state_t
{
	MS_READY,
	MS_READING,
	MS_DONE,
};

#define SEATALK_SETTINGS_BASE		50
#define NMEA_SETTINGS_BASE			150
#define MAX_DATA_AGE_MS				10000UL
#define KNOTS_IN_MPS				1.9438f
#define KNOTS_IN_KMPH				0.54f

#endif
