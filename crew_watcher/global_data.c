#include "crew_watcher.h"
#include "seatalk.h"

volatile unsigned char rf_read_ready=FALSE;
char text_line[17];
volatile char nmea_messages[NUMBER_NMEA_MESSAGES][NMEA_MESSAGE_MAX_LENGTH+1];
volatile unsigned long one_second_tick_count=100UL;
volatile unsigned char tick=0;
char next_message_field[83];
float latitude_degrees;
float longitude_degrees;
float latitude_minutes;
float longitude_minutes;
unsigned long latitude_timestamp=0L;
unsigned long longitude_timestamp=0L;
unsigned long last_alarm_accept_time=0L;
char number_buffer[8];
char degree[2]={223, 0};
volatile unsigned char buttons=0;
volatile unsigned int c=0; 
unsigned char channel_times[CHANNELS]={0, 0, 0, 0};
unsigned char channel_config[CHANNELS];
unsigned char alarm_time;
app_state_t app_state=MONITORING;
unsigned char alarm_channel;
float alarm_latitude_degrees;
float alarm_longitude_degrees;
float alarm_latitude_minutes;
float alarm_longitude_minutes;
unsigned char alarm_pos_available;
settings_state_t settings_state;
unsigned char main_menu_position;
unsigned char channel_menu_position;
unsigned char backlight;
unsigned char position_display_time;
unsigned char settings_display_time;
volatile unsigned char seatalk_messages[SEATALK_NUMBER_OF_MESSAGES][SEATALK_MAX_MESSAGE_SIZE+1];
message_handler handler;
unsigned char seatalk_sentence[18];
switch_state_t alarm_output_normal_state;

volatile unsigned char seatalk_byte_to_write;
volatile unsigned char seatalk_command_bit;
volatile unsigned char seatalk_transmit_state;
