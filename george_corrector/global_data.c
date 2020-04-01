#include "seatalk.h"
#include "george_corrector.h"

volatile unsigned char seatalk_messages[SEATALK_NUMBER_OF_MESSAGES][SEATALK_MAX_MESSAGE_SIZE+1];
int heading=0;
message_handler handler;
volatile unsigned long tick_count=0UL;
volatile unsigned char tick=0;
unsigned char autopilot_remote_command=APR_NONE;
int auto_course=0;
unsigned long last_standby_time=0L;
unsigned char auto_received=FALSE;
int autopilot_followed_course=0;
unsigned char autopilot_state=APS_UNKNOWN;
unsigned char autopilot_required_state=APS_UNKNOWN;