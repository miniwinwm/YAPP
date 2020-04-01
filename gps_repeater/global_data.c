#include "gps_repeater.h"

volatile char nmea_messages[NUMBER_NMEA_MESSAGES][NMEA_MESSAGE_MAX_LENGTH+1];
volatile unsigned long one_second_tick_count=100UL;
volatile unsigned char tick=0;
char next_message_field[83];
float latitude_dm;
float longitude_dm;
float latitude_dd;
float longitude_dd;
float sog;
float cog;
unsigned long position_timestamp=0L;
unsigned long cog_sog_timestamp=0L;
char number_buffer[8];
char degree[2]={223, 0};