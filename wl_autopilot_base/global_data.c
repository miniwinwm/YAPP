#include "wl_autopilot_base.h"
#include "seatalk.h"

volatile unsigned char rf_read_ready=FALSE;
volatile unsigned char seatalk_messages[SEATALK_NUMBER_OF_MESSAGES][SEATALK_MAX_MESSAGE_SIZE+1];
message_handler handler;
unsigned char seatalk_sentence[18];
volatile unsigned char seatalk_byte_to_write;
volatile unsigned char seatalk_command_bit;
volatile unsigned char seatalk_transmit_state;
