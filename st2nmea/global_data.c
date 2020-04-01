#include "st2nmea.h"
#include "seatalk.h"
#include "nmea.h"
#include "util.h"

#pragma udata DATA 

#ifdef _DEBUG
char debug_string[50];
#endif

unsigned long next_depth_send_time=0UL;
unsigned long next_boatspeed_send_time=0UL;
unsigned long next_comp_rudd_send_time=0UL;
unsigned long next_variation_send_time=0UL;
unsigned long next_temperature_send_time=0UL;
unsigned long next_triplog_send_time=0UL;
unsigned long next_trip_send_time=0UL;
unsigned long next_log_send_time=0UL;
unsigned long next_apparent_wind_angle_send_time=0UL;
unsigned long next_apparent_wind_speed_send_time=0UL;
unsigned long next_sog_send_time=0UL;
unsigned long next_cog_send_time=0UL;
unsigned long next_gmt_send_time=0UL;
unsigned long next_date_send_time=0UL;
unsigned long next_latitude_send_time=0UL;
unsigned long next_longitude_send_time=0UL;

// seatalk add more data variables here
float seatalk_depth;
unsigned long seatalk_depth_receive_time=0UL;
float seatalk_boatspeed;
unsigned long seatalk_boatspeed_receive_time=0UL;
float seatalk_heading_magnetic;
unsigned long seatalk_heading_magnetic_receive_time=0UL;
float seatalk_rudder;
unsigned long seatalk_rudder_receive_time=0UL;
float seatalk_variation;
unsigned long seatalk_variation_receive_time=0UL;
float seatalk_temperature;
unsigned long seatalk_temperature_receive_time=0UL;
float seatalk_trip;
unsigned long seatalk_trip_receive_time=0UL;
float seatalk_log;
unsigned long seatalk_log_receive_time=0UL;
float seatalk_apparent_wind_angle;
unsigned long seatalk_apparent_wind_angle_receive_time=0UL;
float seatalk_apparent_wind_speed;
unsigned long seatalk_apparent_wind_speed_receive_time=0UL;
float seatalk_sog;
unsigned long seatalk_sog_receive_time=0UL;
float seatalk_cog;
unsigned long seatalk_cog_receive_time=0UL;
signed int seatalk_latitude_degrees;
float seatalk_latitude_minutes;
unsigned long seatalk_latitude_receive_time=0UL;
signed int seatalk_longitude_degrees;
float seatalk_longitude_minutes;
unsigned long seatalk_longitude_receive_time=0UL;
time_t seatalk_gmt;
unsigned long seatalk_gmt_receive_time=0UL;
date_t seatalk_date;
unsigned long seatalk_date_receive_time=0UL;

// seatalk add default transmit period settings here
const unsigned char seatalk_default_settings[]={2, 		// depth
												1,		// boatspeed
												1,		// compass+rudder
												10,		// variation
												15, 	// water temp
												3,		// triplog
												0,		// trip
												0,		// log
												2,		// awa
												2,		// aws
												2,		// sog
												2,		// cog
												2,		// latitude
												2,		// longitude
												2,		// GMT
												10,		// date
											   };

// nmea add more data variables here
float nmea_depth;
unsigned long nmea_depth_receive_time=0UL;
float nmea_boatspeed;
unsigned long nmea_boatspeed_receive_time=0UL;
float nmea_heading_magnetic;
unsigned long nmea_heading_magnetic_receive_time=0UL;
float nmea_heading_true;
unsigned long nmea_heading_true_receive_time=0UL;
float nmea_rudder;
unsigned long nmea_rudder_receive_time=0UL;
float nmea_variation;
unsigned long nmea_variation_receive_time=0UL;
float nmea_temperature;
unsigned long nmea_temperature_receive_time=0UL;
float nmea_trip;
unsigned long nmea_trip_receive_time=0UL;
float nmea_log;
unsigned long nmea_log_receive_time=0UL;
float nmea_apparent_wind_angle;
unsigned long nmea_apparent_wind_angle_receive_time=0UL;
float nmea_apparent_wind_speed;
unsigned long nmea_apparent_wind_speed_receive_time=0UL;
time_t nmea_time;
unsigned long nmea_time_receive_time=0UL;
date_t nmea_date;
unsigned long nmea_date_receive_time=0UL;
float nmea_sog;
unsigned long nmea_sog_receive_time=0UL;
float nmea_cog;
unsigned long nmea_cog_receive_time=0UL;
signed int nmea_latitude_degrees;
float nmea_latitude_minutes;
unsigned long nmea_latitude_receive_time=0UL;
signed int nmea_longitude_degrees;
float nmea_longitude_minutes;
unsigned long nmea_longitude_receive_time=0UL;

unsigned long next_dpt_send_time=0UL;
unsigned long next_dbt_send_time=0UL;
unsigned long next_vhw_send_time=0UL;
unsigned long next_rsa_send_time=0UL;
unsigned long next_hdm_send_time=0UL;
unsigned long next_hdg_send_time=0UL;
unsigned long next_hdt_send_time=0UL;
unsigned long next_mtw_send_time=0UL;
unsigned long next_vlw_send_time=0UL;
unsigned long next_vwr_send_time=0UL;
unsigned long next_vwt_send_time=0UL;
unsigned long next_mwv_send_time=0UL;
unsigned long next_rmc_send_time=0UL;
unsigned long next_gll_send_time=0UL;

// nmea add default transmit period settings here
const unsigned char nmea_default_settings[]={2, 		// DPT
											 2, 		// DBT
											 2,			// VHW
											 2,			// RSA
											 2,			// HDM
											 2,			// HDG
											 2,			// HDT
											 2,			// MTW
											 2,			// VLW
											 2,			// VWR
											 2,			// VWT
											 2,			// MWV
											 2,			// RMC
											 2,			// GLL
											};		 				

volatile char nmea_messages_in[NUMBER_NMEA_MESSAGES][NMEA_MESSAGE_STORAGE];
volatile char nmea_messages_out[NMEA_OUT_BUFFER_SIZE];
volatile unsigned int nmea_out_next_read_pos=0;
volatile unsigned int nmea_out_next_write_pos=0;
volatile unsigned int nmea_out_space=NMEA_OUT_BUFFER_SIZE;
char next_message_field[NMEA_MAX_MESSAGE_LENGTH+1];
char nmea_message[NMEA_MAX_MESSAGE_LENGTH+1];
volatile unsigned char seatalk_messages_in[SEATALK_NUMBER_OF_MESSAGES_IN][SEATALK_MAX_MESSAGE_SIZE+1];
unsigned char seatalk_messages_out[SEATALK_NUMBER_OF_MESSAGES_OUT][SEATALK_MAX_MESSAGE_SIZE];
seatalk_message_handler handler;
unsigned char seatalk_sentence[SEATALK_MAX_MESSAGE_SIZE];
volatile unsigned char seatalk_byte_to_write;
volatile unsigned char seatalk_command_bit;
volatile unsigned char seatalk_transmit_state;
unsigned char seatalk_out_next_read_pos=0;
unsigned char seatalk_out_next_write_pos=0;
unsigned char seatalk_out_space=SEATALK_NUMBER_OF_MESSAGES_OUT;
volatile unsigned long millisecond_tick_count=60000UL;
const unsigned char seatalk_default_settings_size=sizeof(seatalk_default_settings)/sizeof(seatalk_default_settings[0]);
unsigned char seatalk_settings[sizeof(seatalk_default_settings)/sizeof(seatalk_default_settings[0])];
const unsigned char nmea_default_settings_size=sizeof(nmea_default_settings)/sizeof(nmea_default_settings[0]);
unsigned char nmea_settings[sizeof(nmea_default_settings)/sizeof(nmea_default_settings[0])];
nav_data_t nav_data;
