#include "seatalk_repeater_16x2.h"
#include "seatalk.h"

#pragma udata DATA 

unsigned char top_line_changed=FALSE;
unsigned char bottom_line_changed=FALSE;
unsigned long last_top_button_press_time=0UL;
unsigned long last_bottom_button_press_time=0UL;
display_t current_display_top=DEPTH;
display_t current_display_bottom=APPARENT_WIND;
char text_line[17];
message_handler handler;
unsigned char backlight=BACKLIGHT_MAX;
volatile unsigned char seatalk_messages_in[SEATALK_NUMBER_OF_MESSAGES_IN][SEATALK_MAX_MESSAGE_SIZE+1];
volatile unsigned long one_second_tick_count=100UL;
volatile unsigned char tick=FALSE;
volatile unsigned char buttons=0;
volatile unsigned int c=0; 
char number_buffer[8];

float seatalk_depth;
float seatalk_boatspeed;
float seatalk_heading_magnetic;
float seatalk_rudder;
float seatalk_variation;
float seatalk_temperature;
float seatalk_trip;
float seatalk_log;
float seatalk_apparent_wind_angle;
float seatalk_apparent_wind_speed;
float seatalk_sog;
float seatalk_cog;
signed int seatalk_latitude_degrees;
float seatalk_latitude_minutes;
signed int seatalk_longitude_degrees;
float seatalk_longitude_minutes;
time_t seatalk_gmt;
date_t seatalk_date;
float seatalk_distance_to_destination;
float seatalk_bearing_to_destination;

unsigned long depth_received_time;
unsigned long boatspeed_received_time;
unsigned long heading_mag_received_time;
unsigned long variation_received_time;
unsigned long trip_received_time;
unsigned long log_received_time;
unsigned long awa_received_time;
unsigned long aws_received_time;
unsigned long sog_received_time;
unsigned long cog_received_time;
unsigned long lat_received_time;
unsigned long long_received_time;
unsigned long time_received_time;
unsigned long date_received_time;
unsigned long temperature_received_time;
unsigned long nav_data_received_time;
