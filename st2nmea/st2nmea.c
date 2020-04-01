#include <p18cxxx.h>
#include <delays.h>
#include <string.h>
#include <float.h>
#include <limits.h>
#include "serial.h"
#include "nmea.h"
#include "st2nmea.h"
#include "seatalk.h"
#include "eeprom.h"
#include "util.h"

// Configuration bits are described in section 24.1
// Note: For a complete list of the available config pragmas and their values, 
// click Help|Topics then select "PIC18 Config Settings" in the Language Tools section.

#pragma config FOSC=INTIO67		// Internal oscillator block, port function on RA6 and RA7 
#pragma config PLLCFG=OFF		// Oscillator PLL set in software for HFINTOSC  
#pragma config PRICLKEN=OFF		// Primary clock disabled, using INTOSC  
#pragma config FCMEN=OFF		// Fail-Safe Clock Monitor disabled  
#pragma config IESO=OFF			// Oscillator Switchover mode disabled  
#pragma config PWRTEN=OFF		// Power up timer disabled  
#pragma config BOREN=SBORDIS	// Brown-out Reset enabled in hardware only (SBOREN is disabled) 
#pragma config BORV=285			// VBOR set to 2.85 V nominal  
#pragma config WDTEN=ON			// Watch dog timer is always enabled.
#pragma config WDTPS=1024		// Watchdog timer prescalar 
#pragma config CCP2MX=PORTC1	// CCP2 input/output is multiplexed with RC1  
#pragma config PBADEN=OFF		// PORTB<5:0> pins are configured as digital I/O on Reset  
#pragma config CCP3MX=PORTC6	// P3A/CCP3 input/output is mulitplexed with RC6  
#pragma config HFOFST=OFF		// HFINTOSC output and ready status are delayed by the oscillator stable status  
#pragma config T3CMX=PORTC0		// T3CKI is on RC0  
#pragma config P2BMX=PORTC0		// P2B is on RC0  
#pragma config MCLRE=EXTMCLR	// MCLR pin enabled, RE3 input pin disabled  
#pragma config STVREN=OFF		// Stack full/underflow will not cause Reset  
#pragma config LVP=OFF			// Single-Supply ICSP disabled  
#pragma config XINST=OFF		// Instruction set extension and Indexed Addressing mode disabled (Legacy mode)  
#pragma config DEBUG=OFF		// Debug disabled  
#pragma config CP0=OFF			// Block 0 (000800-001FFFh) not code-protected  
#pragma config CP1=OFF			// Block 1 (002000-003FFFh) not code-protected  
#pragma config CP3=OFF			// Block 3 (006000-007FFFh) not code-protected  
#pragma config CPB=OFF			// Boot block (000000-0007FFh) not code-protected  
#pragma config CPD=OFF			// Data EEPROM not code-protected  
#pragma config WRT0=OFF			// Block 0 (000800-001FFFh) not write-protected  
#pragma config WRT1=OFF			// Block 1 (002000-003FFFh) not write-protected  
#pragma config WRT2=OFF			// Block 2 (004000-005FFFh) not write-protected  
#pragma config WRT3=OFF			// Block 3 (006000-007FFFh) not write-protected  
#pragma config WRTC=OFF			// Configuration registers (300000-3000FFh) not write-protected  
#pragma config WRTB=OFF			// Boot Block (000000-0007FFh) not write-protected  
#pragma config WRTD=OFF			// Data EEPROM not write-protected  
#pragma config EBTR0=OFF		// Block 0 (000800-001FFFh) not protected from table reads executed in other blocks  
#pragma config EBTR1=OFF		// Block 1 (002000-003FFFh) not protected from table reads executed in other blocks  
#pragma config EBTR2=OFF		// Block 2 (004000-005FFFh) not protected from table reads executed in other blocks  
#pragma config EBTR3=OFF		// Block 3 (006000-007FFFh) not protected from table reads executed in other blocks  
#pragma config EBTRB=OFF		// Boot Block (000000-0007FFh) not protected from table reads executed in other blocks  

// for 64MHz operation the following delays can be used
// Delay1TCY(); //delays 1/16us
// Delay10TCYx(16); //delay 10us
// Delay10TCYx(160); //delay 100us
// Delay100TCYx(160); //delay 1ms
// Delay1KTCYx(160); //delay 10ms
// Delay10KTCYx(160); //delay 100ms

#ifdef _DEBUG
#include <stdio.h>
extern char debug_string[];
#endif

extern volatile char nmea_messages_in[NUMBER_NMEA_MESSAGES][NMEA_MESSAGE_STORAGE];
extern volatile char nmea_messages_out[NMEA_OUT_BUFFER_SIZE];
extern volatile unsigned char seatalk_messages_in[SEATALK_NUMBER_OF_MESSAGES_IN][SEATALK_MAX_MESSAGE_SIZE+1];
extern volatile unsigned int nmea_out_next_read_pos;
extern volatile unsigned int nmea_out_next_write_pos;
extern volatile unsigned int nmea_out_space;
extern volatile unsigned long millisecond_tick_count;
extern volatile unsigned char seatalk_command_bit;
extern volatile unsigned char seatalk_transmit_state;
extern volatile unsigned char seatalk_byte_to_write;
extern const unsigned char seatalk_default_settings[];
extern unsigned char seatalk_settings[];
extern const unsigned char seatalk_default_settings_size;
extern const unsigned char nmea_default_settings[];
extern unsigned char nmea_settings[];
extern const unsigned char nmea_default_settings_size;
extern nav_data_t nav_data;

// seatalk add more data variables here
extern unsigned long next_depth_send_time;
extern unsigned long next_boatspeed_send_time;
extern unsigned long next_comp_rudd_send_time;
extern unsigned long next_variation_send_time;
extern unsigned long next_temperature_send_time;
extern unsigned long next_triplog_send_time;
extern unsigned long next_trip_send_time;
extern unsigned long next_log_send_time;
extern unsigned long next_apparent_wind_angle_send_time;
extern unsigned long next_apparent_wind_speed_send_time;
extern unsigned long next_sog_send_time;
extern unsigned long next_cog_send_time;
extern unsigned long next_gmt_send_time;
extern unsigned long next_date_send_time;
extern unsigned long next_latitude_send_time;
extern unsigned long next_longitude_send_time;

extern float seatalk_depth;
extern unsigned long seatalk_depth_receive_time;
extern float seatalk_boatspeed;
extern unsigned long seatalk_boatspeed_receive_time;
extern float seatalk_heading_magnetic;
extern unsigned long seatalk_heading_magnetic_receive_time;
extern float seatalk_heading_magnetic;
extern unsigned long seatalk_heading_magnetic_receive_time;
extern float seatalk_rudder;
extern unsigned long seatalk_rudder_receive_time;
extern float seatalk_variation;
extern long seatalk_variation_receive_time;
extern float seatalk_temperature;
extern long seatalk_temperature_receive_time;
extern float seatalk_trip;
extern long seatalk_trip_receive_time;
extern float seatalk_log;
extern long seatalk_log_receive_time;
extern float seatalk_apparent_wind_angle;
extern unsigned long seatalk_apparent_wind_angle_receive_time;
extern float seatalk_apparent_wind_speed;
extern unsigned long seatalk_apparent_wind_speed_receive_time;
extern float seatalk_sog;
extern unsigned long seatalk_sog_receive_time;
extern float seatalk_cog;
extern unsigned long seatalk_cog_receive_time;
extern signed int seatalk_latitude_degrees;
extern float seatalk_latitude_minutes;
extern unsigned long seatalk_latitude_receive_time;
extern signed int seatalk_longitude_degrees;
extern float seatalk_longitude_minutes;
extern unsigned long seatalk_longitude_receive_time;
extern time_t seatalk_gmt;
extern unsigned long seatalk_gmt_receive_time;
extern date_t seatalk_date;
extern unsigned long seatalk_date_receive_time;

// nmea add more datatypes here
extern unsigned long next_dpt_send_time;
extern unsigned long next_dbt_send_time;
extern unsigned long next_vhw_send_time;
extern unsigned long next_rsa_send_time;
extern unsigned long next_hdm_send_time;
extern unsigned long next_hdg_send_time;
extern unsigned long next_hdt_send_time;
extern unsigned long next_mtw_send_time;
extern unsigned long next_vlw_send_time;
extern unsigned long next_vwr_send_time;
extern unsigned long next_vwt_send_time;
extern unsigned long next_mwv_send_time;
extern unsigned long next_rmc_send_time;
extern unsigned long next_gll_send_time;

extern float nmea_depth;
extern unsigned long nmea_depth_receive_time;
extern float nmea_boatspeed;
extern unsigned long nmea_boatspeed_receive_time;
extern float nmea_heading_magnetic;
extern unsigned long nmea_heading_magnetic_receive_time;
extern float nmea_rudder;
extern unsigned long nmea_rudder_receive_time;
extern float nmea_variation;
extern unsigned long nmea_variation_receive_time;
extern float nmea_temperature;
extern unsigned long nmea_temperature_receive_time;
extern float nmea_trip;
extern unsigned long nmea_trip_receive_time;
extern float nmea_log;
extern unsigned long nmea_log_receive_time;
extern float nmea_apparent_wind_angle;
extern unsigned long nmea_apparent_wind_angle_receive_time;
extern float nmea_apparent_wind_speed;
extern unsigned long nmea_apparent_wind_speed_receive_time;
extern time_t nmea_time;
extern unsigned long nmea_time_receive_time;
extern date_t nmea_date;
extern unsigned long nmea_date_receive_time;
extern float nmea_sog;
extern unsigned long nmea_sog_receive_time;
extern float nmea_cog;
extern unsigned long nmea_cog_receive_time;
extern signed int nmea_latitude_degrees;
extern float nmea_latitude_minutes;
extern unsigned long nmea_latitude_receive_time;
extern signed int nmea_longitude_degrees;
extern float nmea_longitude_minutes;
extern unsigned long nmea_longitude_receive_time;

void low_isr(void);
static void init(void);
static void seatalk_message_handler(unsigned char message_type);
static void queue_next_nmea_message(void);
static void queue_next_seatalk_message(void);
static void clear_watchdog(void);

void main(void) 
{ 
	init();

/*
strcpypgm2ram(debug_string, "STN\r\n");
nmea_queue_message_to_send(debug_string);
*/
	
	while(1)		
	{
		clear_watchdog();
				
		// process next waiting received nmea message
		nmea_process_next_message();

		// process next waiting received seatalk message
		seatalk_process_next_message();

		// queue next nmea message
		queue_next_nmea_message();
	
		// queue next seatalk messages
		queue_next_seatalk_message();

		// send next queued nmea message
		// nothing to do, handled by interrupts automatically

		// send next queued seatalk message 
		seatalk_send_next_message();
	}
} 

static void clear_watchdog(void)
{
	ClrWdt();
}

static void queue_next_seatalk_message(void)
{
	unsigned char do_send;

	// SEATALK_ID_DEPTH
	if(millisecond_tick_count>next_depth_send_time && seatalk_settings[SEATALK_DEPTH]>0)
	{
		if(millisecond_tick_count-nmea_depth_receive_time<MAX_DATA_AGE_MS)
		{
			seatalk_depth_send(nmea_depth);
		}
		next_depth_send_time=millisecond_tick_count+1000UL*(unsigned long)seatalk_settings[SEATALK_DEPTH];
	}

	// SEATALK_ID_BOATSPEED
	if(millisecond_tick_count>next_boatspeed_send_time && seatalk_settings[SEATALK_BOATSPEED]>0)
	{
		if(millisecond_tick_count-nmea_boatspeed_receive_time<MAX_DATA_AGE_MS)
		{
			seatalk_boatspeed_send(nmea_boatspeed);
		}
		next_boatspeed_send_time=millisecond_tick_count+1000UL*(unsigned long)seatalk_settings[SEATALK_BOATSPEED];
	}

	// SEATALK_ID_COMP_RUDD
	if(millisecond_tick_count>next_comp_rudd_send_time && seatalk_settings[SEATALK_COMP_RUDD]>0)
	{
		do_send=0;

		if(millisecond_tick_count-nmea_heading_magnetic_receive_time<MAX_DATA_AGE_MS)
		{
			do_send++;
		}
		else
		{
			nmea_heading_magnetic=FLT_MAX;
		}

		if(millisecond_tick_count-nmea_rudder_receive_time<MAX_DATA_AGE_MS)
		{
			do_send++;
		}
		else
		{
			nmea_rudder=FLT_MAX;
		}

		if(do_send)
		{
			seatalk_compass_rudder_send(nmea_heading_magnetic, nmea_rudder);
		}

		next_comp_rudd_send_time=millisecond_tick_count+1000UL*(unsigned long)seatalk_settings[SEATALK_COMP_RUDD];
	}

	// SEATALK_ID_VARIATION
	if(millisecond_tick_count>next_variation_send_time && seatalk_settings[SEATALK_VARIATION]>0)
	{
		if(millisecond_tick_count-nmea_variation_receive_time<MAX_DATA_AGE_MS)
		{
			seatalk_variation_send(nmea_variation);
		}
		next_variation_send_time=millisecond_tick_count+1000UL*(unsigned long)seatalk_settings[SEATALK_VARIATION];
	}

	// SEATALK_ID_TEMPERATURE
	if(millisecond_tick_count>next_temperature_send_time && seatalk_settings[SEATALK_TEMPERATURE]>0)
	{
		if(millisecond_tick_count-nmea_temperature_receive_time<MAX_DATA_AGE_MS)
		{
			seatalk_temperature_send(nmea_temperature);
		}
		next_temperature_send_time=millisecond_tick_count+1000UL*(unsigned long)seatalk_settings[SEATALK_TEMPERATURE];
	}
	
	// SEATALK_ID_TRIPLOG	
	if(millisecond_tick_count>next_triplog_send_time && seatalk_settings[SEATALK_TRIPLOG]>0)
	{
		do_send=0;
				
		if(millisecond_tick_count-nmea_trip_receive_time<MAX_DATA_AGE_MS)
		{
			do_send++;
		}
		else
		{
			nmea_trip=FLT_MAX;
		}
		
		if(millisecond_tick_count-nmea_log_receive_time<MAX_DATA_AGE_MS)
		{
			do_send++;
		}
		else
		{
			nmea_log=FLT_MAX;
		}
		
		if(do_send)
		{
			seatalk_triplog_send(nmea_trip, nmea_log);
		}
		
		next_triplog_send_time=millisecond_tick_count+1000UL*(unsigned long)seatalk_settings[SEATALK_TRIPLOG];
	}
	
	// SEATALK_ID_TRIP
	if(millisecond_tick_count>next_trip_send_time && seatalk_settings[SEATALK_TRIP]>0)
	{
		if(millisecond_tick_count-nmea_trip_receive_time<MAX_DATA_AGE_MS)
		{
			seatalk_trip_send(nmea_trip);
		}
		next_trip_send_time=millisecond_tick_count+1000UL*(unsigned long)seatalk_settings[SEATALK_TRIP];
	}

	// SEATALK_ID_LOG
	if(millisecond_tick_count>next_log_send_time && seatalk_settings[SEATALK_LOG]>0)
	{
		if(millisecond_tick_count-nmea_log_receive_time<MAX_DATA_AGE_MS)
		{
			seatalk_log_send(nmea_log);
		}
		next_log_send_time=millisecond_tick_count+1000UL*(unsigned long)seatalk_settings[SEATALK_LOG];
	}
	
	// SEATALK_ID_APPARENT_WIND_ANGLE
	if(millisecond_tick_count>next_apparent_wind_angle_send_time && seatalk_settings[SEATALK_APPARENT_WIND_ANGLE]>0)
	{
		if(millisecond_tick_count-nmea_apparent_wind_angle_receive_time<MAX_DATA_AGE_MS)
		{
			seatalk_apparent_wind_angle_send(nmea_apparent_wind_angle);
		}
		next_apparent_wind_angle_send_time=millisecond_tick_count+1000UL*(unsigned long)seatalk_settings[SEATALK_APPARENT_WIND_ANGLE];
	}
	
	// SEATALK_ID_APPARENT_WIND_SPEED
	if(millisecond_tick_count>next_apparent_wind_speed_send_time && seatalk_settings[SEATALK_APPARENT_WIND_SPEED]>0)
	{
		if(millisecond_tick_count-nmea_apparent_wind_speed_receive_time<MAX_DATA_AGE_MS)
		{
			seatalk_apparent_wind_speed_send(nmea_apparent_wind_speed);
		}
		next_apparent_wind_speed_send_time=millisecond_tick_count+1000UL*(unsigned long)seatalk_settings[SEATALK_APPARENT_WIND_SPEED];
	}
	
	// SEATALK_ID_SOG
	if(millisecond_tick_count>next_sog_send_time && seatalk_settings[SEATALK_SOG]>0)
	{
		if(millisecond_tick_count-nmea_sog_receive_time<MAX_DATA_AGE_MS)
		{
			seatalk_sog_send(nmea_sog);
		}
		next_sog_send_time=millisecond_tick_count+1000UL*(unsigned long)seatalk_settings[SEATALK_SOG];
	}
	
	// SEATALK_ID_COG
	if(millisecond_tick_count>next_cog_send_time && seatalk_settings[SEATALK_COG]>0)
	{
		if(millisecond_tick_count-nmea_cog_receive_time<MAX_DATA_AGE_MS)
		{
			seatalk_cog_send(nmea_cog);
		}
		next_cog_send_time=millisecond_tick_count+1000UL*(unsigned long)seatalk_settings[SEATALK_COG];
	}

	// SEATALK_ID_GMT
	if(millisecond_tick_count>next_gmt_send_time && seatalk_settings[SEATALK_GMT]>0)
	{
		if(millisecond_tick_count-nmea_time_receive_time<MAX_DATA_AGE_MS)
		{
			seatalk_gmt_send(nmea_time); 
		}
		next_gmt_send_time=millisecond_tick_count+1000UL*(unsigned long)seatalk_settings[SEATALK_GMT];
	}
	
	// SEATALK_ID_DATE
	if(millisecond_tick_count>next_date_send_time && seatalk_settings[SEATALK_DATE]>0)
	{
		if(millisecond_tick_count-nmea_date_receive_time<MAX_DATA_AGE_MS)
		{
			seatalk_date_send(nmea_date); 
		}
		next_date_send_time=millisecond_tick_count+1000UL*(unsigned long)seatalk_settings[SEATALK_DATE];
	}
	
	// SEATALK_ID_LATITUDE
	if(millisecond_tick_count>next_latitude_send_time && seatalk_settings[SEATALK_LATITUDE]>0)
	{
		if(millisecond_tick_count-nmea_latitude_receive_time<MAX_DATA_AGE_MS)
		{	
			seatalk_latitude_send(nmea_latitude_degrees, nmea_latitude_minutes); 
		}		
		next_latitude_send_time=millisecond_tick_count+1000UL*(unsigned long)seatalk_settings[SEATALK_LATITUDE];
	}

	// SEATALK_ID_LONGITUDE
	if(millisecond_tick_count>next_longitude_send_time && seatalk_settings[SEATALK_LONGITUDE]>0)
	{
		if(millisecond_tick_count-nmea_longitude_receive_time<MAX_DATA_AGE_MS)
		{
			seatalk_longitude_send(nmea_longitude_degrees, nmea_longitude_minutes); 
		}
		next_longitude_send_time=millisecond_tick_count+1000UL*(unsigned long)seatalk_settings[SEATALK_LONGITUDE];
	}
			
	// seatalk add more message types here
}

static void queue_next_nmea_message(void)
{
	float heading_true;
	unsigned char do_send;
	signed int bearing;

	// DPT
	if(millisecond_tick_count>next_dpt_send_time && nmea_settings[NMEA_DPT]>0)
	{
		if(millisecond_tick_count-seatalk_depth_receive_time<MAX_DATA_AGE_MS)
		{
			nmea_DPT_send(seatalk_depth);
		}
		next_dpt_send_time=millisecond_tick_count+1000UL*(unsigned long)nmea_settings[NMEA_DPT];
	}

	// DBT
	if(millisecond_tick_count>next_dbt_send_time  && nmea_settings[NMEA_DBT]>0)
	{
		if(millisecond_tick_count-seatalk_depth_receive_time<MAX_DATA_AGE_MS)
		{
			nmea_DBT_send(seatalk_depth);
		}
		next_dbt_send_time=millisecond_tick_count+1000UL*(unsigned long)nmea_settings[NMEA_DBT];
	}

	// VHW
	if(millisecond_tick_count>next_vhw_send_time  && nmea_settings[NMEA_VHW]>0)
	{
		do_send=0;

		if(millisecond_tick_count-seatalk_boatspeed_receive_time<MAX_DATA_AGE_MS)
		{
			do_send++;
		}
		else
		{
			seatalk_boatspeed=FLT_MAX;
		}
		if(millisecond_tick_count-seatalk_heading_magnetic_receive_time<MAX_DATA_AGE_MS &&
			millisecond_tick_count-seatalk_variation_receive_time<MAX_DATA_AGE_MS)
		{
			heading_true=util_calc_heading_true(seatalk_heading_magnetic, seatalk_variation);
			do_send++;
		}
		else 
		{
			heading_true=FLT_MAX;
		} 
		if(millisecond_tick_count-seatalk_heading_magnetic_receive_time<MAX_DATA_AGE_MS)
		{
			do_send++;
		}
		else
		{
			seatalk_heading_magnetic=FLT_MAX;
		}
		if(do_send>0)
		{  
			nmea_VHW_send(heading_true, seatalk_heading_magnetic, seatalk_boatspeed);
		}
		next_vhw_send_time=millisecond_tick_count+1000UL*(unsigned long)nmea_settings[NMEA_VHW];
	}		

	// RSA
	if(millisecond_tick_count>next_rsa_send_time  && nmea_settings[NMEA_RSA]>0)
	{
		if(millisecond_tick_count-seatalk_rudder_receive_time<MAX_DATA_AGE_MS)
		{
			nmea_RSA_send(seatalk_rudder);
		}
		next_rsa_send_time=millisecond_tick_count+1000UL*(unsigned long)nmea_settings[NMEA_RSA];
	}

	// HDM
	if(millisecond_tick_count>next_hdm_send_time && nmea_settings[NMEA_HDM]>0)
	{
		if(millisecond_tick_count-seatalk_heading_magnetic_receive_time<MAX_DATA_AGE_MS)
		{
			nmea_HDM_send(seatalk_heading_magnetic);
		}
		next_hdm_send_time=millisecond_tick_count+1000UL*(unsigned long)nmea_settings[NMEA_HDM];
	}

	// HDG
	if(millisecond_tick_count>next_hdg_send_time && nmea_settings[NMEA_HDG]>0)
	{
		do_send=0;

		if(millisecond_tick_count-seatalk_heading_magnetic_receive_time<MAX_DATA_AGE_MS)
		{
			do_send++;
		}
		else
		{
			seatalk_heading_magnetic=FLT_MAX;
		}
		if(millisecond_tick_count-seatalk_variation_receive_time<MAX_DATA_AGE_MS)
		{
			do_send++;
		}
		else
		{
			seatalk_variation=FLT_MAX;
		}
		if(do_send>0)
		{
			nmea_HDG_send(seatalk_heading_magnetic, seatalk_variation);
		}

		next_hdg_send_time=millisecond_tick_count+1000UL*(unsigned long)nmea_settings[NMEA_HDG];
	}

	// HDT
	if(millisecond_tick_count>next_hdt_send_time && nmea_settings[NMEA_HDT]>0)
	{
		if(millisecond_tick_count-seatalk_heading_magnetic_receive_time<MAX_DATA_AGE_MS &&
			millisecond_tick_count-seatalk_variation_receive_time<MAX_DATA_AGE_MS)
		{
			heading_true=util_calc_heading_true(seatalk_heading_magnetic, seatalk_variation);
			nmea_HDT_send(heading_true);
		}

		next_hdt_send_time=millisecond_tick_count+1000UL*(unsigned long)nmea_settings[NMEA_HDT];
	}
	
	// MTW
	if(millisecond_tick_count>next_mtw_send_time && nmea_settings[NMEA_MTW]>0)
	{
		if(millisecond_tick_count-seatalk_temperature_receive_time<MAX_DATA_AGE_MS)
		{
			nmea_MTW_send(seatalk_temperature);
		}
		next_mtw_send_time=millisecond_tick_count+1000UL*(unsigned long)nmea_settings[NMEA_MTW];
	}
	
	// VLW
	if(millisecond_tick_count>next_vlw_send_time && nmea_settings[NMEA_VLW]>0)
	{
		do_send=0;

		if(millisecond_tick_count-seatalk_trip_receive_time<MAX_DATA_AGE_MS)
		{
			do_send++;
		}
		else
		{
			seatalk_trip=FLT_MAX;
		}
		if(millisecond_tick_count-seatalk_log_receive_time<MAX_DATA_AGE_MS)
		{
			do_send++;
		}
		else
		{
			seatalk_log=FLT_MAX;
		}
		if(do_send>0)
		{
			nmea_VLW_send(seatalk_trip, seatalk_log);
		}

		next_vlw_send_time=millisecond_tick_count+1000UL*(unsigned long)nmea_settings[NMEA_VLW];
	}
	
	// VWR
	if(millisecond_tick_count>next_vwr_send_time && nmea_settings[NMEA_VWR]>0)
	{
		do_send=0;

		if(millisecond_tick_count-seatalk_apparent_wind_angle_receive_time<MAX_DATA_AGE_MS)
		{
			do_send++;
		}
		else
		{
			seatalk_apparent_wind_angle=FLT_MAX;
		}
		if(millisecond_tick_count-seatalk_apparent_wind_speed_receive_time<MAX_DATA_AGE_MS)
		{
			do_send++;
		}
		else
		{
			seatalk_apparent_wind_speed=FLT_MAX;
		}
		if(do_send>0)
		{
			nmea_VWR_send(seatalk_apparent_wind_angle, seatalk_apparent_wind_speed);
		}

		next_vwr_send_time=millisecond_tick_count+1000UL*(unsigned long)nmea_settings[NMEA_VWR];
	}
	
	// VWT
	if(millisecond_tick_count>next_vwt_send_time && nmea_settings[NMEA_VWT]>0)
	{
		if(millisecond_tick_count-seatalk_apparent_wind_angle_receive_time<MAX_DATA_AGE_MS &&
			millisecond_tick_count-seatalk_apparent_wind_speed_receive_time<MAX_DATA_AGE_MS &&
			millisecond_tick_count-seatalk_boatspeed_receive_time<MAX_DATA_AGE_MS)
		{
			nmea_VWT_send(seatalk_apparent_wind_angle, seatalk_apparent_wind_speed, seatalk_boatspeed);
		}

		next_vwt_send_time=millisecond_tick_count+1000UL*(unsigned long)nmea_settings[NMEA_VWT];
	}
	
	// MWV
	if(millisecond_tick_count>next_mwv_send_time && nmea_settings[NMEA_MWV]>0)
	{
		do_send=0;

		if(millisecond_tick_count-seatalk_apparent_wind_angle_receive_time<MAX_DATA_AGE_MS)
		{
			do_send++;
		}
		else
		{
			seatalk_apparent_wind_angle=FLT_MAX;
		}
		if(millisecond_tick_count-seatalk_apparent_wind_speed_receive_time<MAX_DATA_AGE_MS)
		{
			do_send++;
		}
		else
		{
			seatalk_apparent_wind_speed=FLT_MAX;
		}
		if(millisecond_tick_count-seatalk_boatspeed_receive_time<MAX_DATA_AGE_MS)
		{
			do_send++;
		}
		else
		{
			seatalk_boatspeed=FLT_MAX;
		}
		if(do_send>0)
		{
			nmea_MWV_send(seatalk_apparent_wind_angle, seatalk_apparent_wind_speed, seatalk_boatspeed);
		}
		next_mwv_send_time=millisecond_tick_count+1000UL*(unsigned long)nmea_settings[NMEA_MWV];
	}
	
	// RMC
	if(millisecond_tick_count>next_rmc_send_time && nmea_settings[NMEA_RMC]>0)
	{
		do_send=0;
		
		if(millisecond_tick_count-seatalk_gmt_receive_time<MAX_DATA_AGE_MS)
		{
			nav_data.time=seatalk_gmt;
			do_send++;
		}	
		else
		{
			nav_data.time.hour=UCHAR_MAX;
		}	
		if(millisecond_tick_count-seatalk_latitude_receive_time<MAX_DATA_AGE_MS)
		{
			nav_data.latitude_degrees=seatalk_latitude_degrees;
			nav_data.latitude_minutes=seatalk_latitude_minutes;
			do_send++;
		}
		else
		{
			nav_data.latitude_degrees=INT_MAX;
		}			
		if(millisecond_tick_count-seatalk_longitude_receive_time<MAX_DATA_AGE_MS)
		{
			nav_data.longitude_degrees=seatalk_longitude_degrees;
			nav_data.longitude_minutes=seatalk_longitude_minutes;
			do_send++;
		}
		else
		{
			nav_data.longitude_degrees=INT_MAX;
		}
		if(millisecond_tick_count-seatalk_sog_receive_time<MAX_DATA_AGE_MS)
		{
			nav_data.sog=seatalk_sog;
			do_send++;
		}
		else
		{
			nav_data.sog=FLT_MAX;
		}
		if(millisecond_tick_count-seatalk_cog_receive_time<MAX_DATA_AGE_MS)
		{
			nav_data.cog=seatalk_cog;
			do_send++;
		}
		else
		{
			nav_data.cog=FLT_MAX;
		}	
		if(millisecond_tick_count-seatalk_date_receive_time<MAX_DATA_AGE_MS)
		{
			nav_data.date=seatalk_date;
			do_send++;
		}	
		else
		{
			nav_data.date.year=UCHAR_MAX;
		}
		if(millisecond_tick_count-seatalk_variation_receive_time<MAX_DATA_AGE_MS)
		{
			nav_data.variation=seatalk_variation;
			do_send++;
		}
		else
		{
			nav_data.variation=FLT_MAX;
		}	
		if(do_send>0)
		{
			nmea_RMC_send(&nav_data);
		}
		next_rmc_send_time=millisecond_tick_count+1000UL*(unsigned long)nmea_settings[NMEA_RMC];
	}	
	
	// GLL
	if(millisecond_tick_count>next_gll_send_time && nmea_settings[NMEA_GLL]>0)
	{
		do_send=0;

		if(millisecond_tick_count-seatalk_latitude_receive_time<MAX_DATA_AGE_MS)
		{
			nav_data.latitude_degrees=seatalk_latitude_degrees;
			nav_data.latitude_minutes=seatalk_latitude_minutes;
			do_send++;
		}
		else
		{
			nav_data.latitude_degrees=INT_MAX;
		}			
		if(millisecond_tick_count-seatalk_longitude_receive_time<MAX_DATA_AGE_MS)
		{
			nav_data.longitude_degrees=seatalk_longitude_degrees;
			nav_data.longitude_minutes=seatalk_longitude_minutes;
			do_send++;
		}
		else
		{
			nav_data.longitude_degrees=INT_MAX;
		}		
		if(millisecond_tick_count-seatalk_gmt_receive_time<MAX_DATA_AGE_MS)
		{
			nav_data.time=seatalk_gmt;
			do_send++;
		}	
		else
		{
			nav_data.time.hour=UCHAR_MAX;
		}	
	
		if(do_send>0)
		{
			nmea_GLL_send(&nav_data);
		}
		next_gll_send_time=millisecond_tick_count+1000UL*(unsigned long)nmea_settings[NMEA_GLL];
	}	
	
	// nmea add more message types here
}

static void init(void)
{
	unsigned char i;

	OSCCONbits.IRCF=6;				// internal oscillator 16MHz, section 2.2.2
	OSCCONbits.IDLEN=0;				// SLEEP enters sleep mode, section 2.2.4
	OSCCONbits.SCS=0;				// system clock determined by config bits, section 2.3
	OSCTUNEbits.PLLEN=1;
	
	serial1_setup();					// initialize serial port

	// set all pins as digital i/o
	ANSELA=0;
	ANSELB=0;
	ANSELC=0;

	for(i=0; i<NUMBER_NMEA_MESSAGES; i++)
	{
		nmea_messages_in[i][0]=MS_DONE;			
	}

	// seatalk
	seatalk_init(seatalk_message_handler);

	// settings
	if(int_EEPROM_getc(0)!='Y' ||		// check settings initialized
		int_EEPROM_getc(1)!='A' ||
		int_EEPROM_getc(2)!='P' ||
		int_EEPROM_getc(3)!='P')
	{	
		// initialize settings
		for(i=0; i<seatalk_default_settings_size; i++)
		{
			int_EEPROM_putc(SEATALK_SETTINGS_BASE+i, seatalk_default_settings[i]);
		}

		for(i=0; i<nmea_default_settings_size; i++)
		{
			int_EEPROM_putc(NMEA_SETTINGS_BASE+i, nmea_default_settings[i]);
		}

		int_EEPROM_putc(0, 'Y');
		int_EEPROM_putc(1, 'A');
		int_EEPROM_putc(2, 'P');
		int_EEPROM_putc(3, 'P');
	}

	for(i=0; i<seatalk_default_settings_size; i++)
	{
		seatalk_settings[i]=int_EEPROM_getc(SEATALK_SETTINGS_BASE+i);
	}

	for(i=0; i<nmea_default_settings_size; i++)
	{
		nmea_settings[i]=int_EEPROM_getc(NMEA_SETTINGS_BASE+i);
	}

    // setup timer 6, this is used for seatalk and the tick timer 
    T6CONbits.TMR6ON=0;			// disable timer 6
    T6CONbits.T6CKPS=0;         // set prescalar is 1
    T6CONbits.T6OUTPS=0;     	// set postscalar to 1
    PR6=0xd1;                   // set timer6 period
    TMR6=0;                     // set timer6
    PIR5bits.TMR6IF=0;          // clear interrupt flag
    T6CONbits.TMR6ON=1;         // enable timer 6

	RCONbits.IPEN=0;	        // disable interrupt priority, section 9.2
    PIE5bits.TMR6IE=1;          // enable timer6 interrupt 
	INTCONbits.PEIE=1;          // enable all unmasked peripheral interrupts
	INTCONbits.GIE=1;           // globally enable interrupts
}		

static void seatalk_message_handler(unsigned char message_type)
{	
	switch(message_type)
	{
		case SEATALK_ID_DEPTH:
			seatalk_depth_receive_time=millisecond_tick_count;	
			break;

		case SEATALK_ID_BOATSPEED:
			seatalk_boatspeed_receive_time=millisecond_tick_count;		
			break;

		case SEATALK_ID_HEADING_MAGNETIC:
			seatalk_heading_magnetic_receive_time=millisecond_tick_count;		
			break;

		case SEATALK_ID_RUDDER:
			seatalk_rudder_receive_time=millisecond_tick_count;		
			break;

		case SEATALK_ID_VARIATION:
			seatalk_variation_receive_time=millisecond_tick_count;		
			break;

		case SEATALK_ID_TEMPERATURE:
			seatalk_temperature_receive_time=millisecond_tick_count;		
			break;
			
		case SEATALK_ID_TRIP:
			seatalk_trip_receive_time=millisecond_tick_count;		
			break;
			
		case SEATALK_ID_LOG:
			seatalk_log_receive_time=millisecond_tick_count;		
			break;
			
		case SEATALK_ID_TRIPLOG:
			seatalk_log_receive_time=millisecond_tick_count;		
			seatalk_trip_receive_time=millisecond_tick_count;		
			break;

		case SEATALK_ID_APPARENT_WIND_ANGLE:
			seatalk_apparent_wind_angle_receive_time=millisecond_tick_count;		
			break;
			
		case SEATALK_ID_APPARENT_WIND_SPEED:
			seatalk_apparent_wind_speed_receive_time=millisecond_tick_count;		
			break;

		case SEATALK_ID_SOG:
			seatalk_sog_receive_time=millisecond_tick_count;		
			break;
			
		case SEATALK_ID_COG:
			seatalk_cog_receive_time=millisecond_tick_count;		
			break;
			
		case SEATALK_ID_LATITUDE:
			seatalk_latitude_receive_time=millisecond_tick_count;		
			break;
			
		case SEATALK_ID_LONGITUDE:
			seatalk_longitude_receive_time=millisecond_tick_count;		
			break;	
			
		case SEATALK_ID_GMT:
			seatalk_gmt_receive_time=millisecond_tick_count;		
			break;
			
		case SEATALK_ID_DATE:
			seatalk_date_receive_time=millisecond_tick_count;		
			break;
									
		// add more seatalk handled messages here

		default:
			break;
	}
}

#pragma code low_vector=0x08
void low_interrupt(void)
{
	_asm GOTO low_isr _endasm
}
	
#pragma code

#pragma interruptlow low_isr 
void low_isr(void)
{
	static unsigned char nmea_in_current_writing_message=0;
	static unsigned char nmea_in_next_writing_position=0;
	static unsigned char nmea_in_waiting_for_message_to_start=TRUE;
	unsigned char read_byte;
	static unsigned char millisecond_timer_counter=0;
	
	// check tick timer
	if(PIE5bits.TMR6IE && PIR5bits.TMR6IF)
	{
		PIR5bits.TMR6IF=0;

		do_seatalk_read();
		do_seatalk_write();
		
		millisecond_timer_counter++;	

		if(millisecond_timer_counter==38)
		{
			millisecond_tick_count++;
			millisecond_timer_counter=0;
		}
	}

	// check uart1 for transmit
	if(PIE1bits.TX1IE && PIR1bits.TX1IF)
	{
		TXREG1=nmea_messages_out[nmea_out_next_read_pos];
		nmea_out_next_read_pos++;
		nmea_out_space++;
		if(nmea_out_next_read_pos==NMEA_OUT_BUFFER_SIZE)
		{
			nmea_out_next_read_pos=0;
		}

		if(nmea_out_next_read_pos==nmea_out_next_write_pos)
		{
			PIE1bits.TX1IE=0;
		}
	}

	// check uart1 for receive
	if(PIE1bits.RC1IE && PIR1bits.RC1IF)
	{
		read_byte=RCREG1;
		PIR1bits.RC1IF=0;
		
		if(RCSTA1bits.OERR)
		{
			RCSTA1bits.CREN=0;
			RCSTA1bits.CREN=1;
		}	
		
		if(nmea_in_waiting_for_message_to_start)
		{			
			if(read_byte=='$' || read_byte=='#')
			{	
				for(nmea_in_current_writing_message=0; nmea_in_current_writing_message<NUMBER_NMEA_MESSAGES; nmea_in_current_writing_message++)
				{
					if(nmea_messages_in[nmea_in_current_writing_message][0]==MS_DONE)
					{
						nmea_in_waiting_for_message_to_start=FALSE;
						nmea_messages_in[nmea_in_current_writing_message][1]=read_byte;
						nmea_in_next_writing_position=2;							
						break;
					}	
				}
			}	
		}	
		else
		{			
			if(nmea_in_next_writing_position<NMEA_MESSAGE_STORAGE-1)
			{
				nmea_messages_in[nmea_in_current_writing_message][nmea_in_next_writing_position]=read_byte;							
				if(read_byte=='\r')
				{				
					nmea_messages_in[nmea_in_current_writing_message][nmea_in_next_writing_position+1]='\0';											
					nmea_messages_in[nmea_in_current_writing_message][0]=MS_READY;
					nmea_in_waiting_for_message_to_start=TRUE;
				}	
				else
				{				
					nmea_in_next_writing_position++;
				}
			}
			else
			{
				// oversize message, abandon
				nmea_in_waiting_for_message_to_start=TRUE;
				nmea_messages_in[nmea_in_current_writing_message][0]=MS_DONE;
			}		
		}	
	}	
}
