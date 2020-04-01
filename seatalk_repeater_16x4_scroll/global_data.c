#include "seatalk.h"
#include "seatalk_repeater_16x4.h"

volatile unsigned char messages[NUMBER_OF_MESSAGES][MAX_MESSAGE_SIZE+1];
float depth=0.0f;
float temperature=0.0f;
float boatspeed=0.0f;
float trip=0.0f;
float boat_log=0.0f;
signed char variation=0;
float tws=0.0f; 
float twa=0.0f;
float awa=0.0f;
float aws=0.0f;
int heading=0;
int heading_true=0;
float sog=0.0f;
unsigned int cog=0;
message_handler handler;
volatile unsigned long tick_count=0UL;
volatile unsigned char tick=0;
unsigned char lamps=0;
unsigned long ttg=0UL;
float distance_to_destination;
unsigned int bearing_to_destination=0;