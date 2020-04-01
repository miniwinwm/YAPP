/** I N C L U D E S **********************************************************/
#include <p18cxxx.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "system\typedefs.h"
#include "system\usb\usb.h"
#include "io_cfg.h"             
#include "user\user.h"
#include "user\seatalk.h"

extern float depth;
extern float temperature;
extern float boatspeed;
extern float trip;
extern float boat_log;
extern signed char variation;
extern signed int latitude_degrees;
extern signed int longitude_degrees;
extern float latitude_minutes;
extern float longitude_minutes;
extern unsigned char hour;
extern unsigned char minute;
extern unsigned char second;
extern unsigned char year;
extern unsigned char month;
extern unsigned char date;
extern float tws;
extern float twa;
extern float sog;
extern float awa;
extern unsigned int cog;
extern float aws;
extern unsigned int heading;
extern float distance_to_destination;
extern unsigned int bearing_to_destination;
extern signed char rudder;
extern char string_buffer1[80];
extern char string_buffer2[80];
extern volatile unsigned long second_tick_count;
extern volatile unsigned long tenth_second_tick_count;
extern unsigned long depth_receive_time;
extern unsigned long awa_receive_time;
extern unsigned long aws_receive_time;
extern unsigned long heading_receive_time;
extern unsigned long boatspeed_receive_time;
extern unsigned long variation_receive_time;
extern unsigned long sog_receive_time;
extern unsigned long cog_receive_time;
extern unsigned long latitude_receive_time;
extern unsigned long longitude_receive_time;
extern unsigned long temperature_receive_time;
extern unsigned long triplog_receive_time;
extern unsigned long nav_to_waypoint_receive_time;
extern unsigned long true_wind_receive_time;
extern unsigned long rudder_receive_time;
extern unsigned long trip_receive_time;
extern unsigned long log_receive_time;

static int abs(int x)
{
	if(x<0)
	{
		return -x;
	}
	return x;
}

static unsigned char calculate_checksum(char *message)
{
	unsigned char checksum=0;
	unsigned char length;
	unsigned char i;

	length=strlen(message)-2;
	for(i=0; i<length; i++)
	{
		checksum^=message[i+1];
	}
	
	return checksum;
}	

void seatalk_message_handler(unsigned char message_type)
{			
	switch(message_type)
	{
		case TRUE_WIND:
			true_wind_receive_time=second_tick_count;
			break;

		case NAV_TO_WAYPOINT:
			nav_to_waypoint_receive_time=second_tick_count;
			break;

		case TRIPLOG:
			triplog_receive_time=second_tick_count;
			break;

		case TEMPERATURE:
			temperature_receive_time=second_tick_count;
			break;

		case VARIATION:
			variation_receive_time=second_tick_count;
			break;

		case DEPTH:
			depth_receive_time=second_tick_count;
			break;

		case SOG:
			sog_receive_time=second_tick_count;
			break;
			
		case COG:
			cog_receive_time=second_tick_count;
			break;
			
		case LATITUDE:
			latitude_receive_time=second_tick_count;
			break;
			
		case LONGITUDE:
			longitude_receive_time=second_tick_count;
			break;
			
		case APPARENT_WIND_ANGLE:
			awa_receive_time=second_tick_count;
			break;
			
		case APPARENT_WIND_SPEED:
			aws_receive_time=second_tick_count;
			break;

		case HEADING:
			heading_receive_time=second_tick_count;
			break;

		case RUDDER:
			rudder_receive_time=second_tick_count;
			break;

		case BOATSPEED:
			boatspeed_receive_time=second_tick_count;
			break;		

		case TRIP:
			trip_receive_time=second_tick_count;
			break;	

		case LOG:
			log_receive_time=second_tick_count;
			break;		
	}
}

void UserInit(void)
{   
	// debug led if fitted
	TRISB &=0b11101111;

	init_seatalk(seatalk_message_handler);

/* sample serial port init if needed
	TRISCbits.TRISC6=1;
	TRISCbits.TRISC7=1;
	SPBRG=155;
	BAUDCONbits.BRG16=0;
	BAUDCONbits.TXCKP=0;
	TXSTAbits.BRGH=0;
	TXSTAbits.SYNC=0;
	TXSTAbits.TX9=0;
	TXSTAbits.TXEN=1;
	RCSTAbits.SPEN=1;
	PIE1bits.TXIE=0;
*/
}

/* sample serial port write if needed
void putstring(char *s)
{
	while(*s)
	{
		while(!TXSTAbits.TRMT);
		TXREG=*s;
		s++;
	}
}
*/

/******************************************************************************
 * Function:        void ProcessIO(void)
 * 
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None 
 *
 * Side Effects:    None
 *
 * Overview:        This function is a place holder for other user routines.155;
 *                  It is a mixture of both USB and non-USB tasks.
 *
 * Note:            None
 *****************************************************************************/
void ProcessIO(void)
{
	static unsigned long int last_send_time=0L;
	static unsigned char next_message_to_send=0;
	unsigned char checksum;

    if((usb_device_state < CONFIGURED_STATE)||(UCONbits.SUSPND==1)) 
	{
		return; 
	}

	parse_next_seatalk_message();

	if(!mUSBUSARTIsTxTrfReady())
	{
		return;
	}

	if(tenth_second_tick_count!=last_send_time)
	{
		last_send_time=tenth_second_tick_count;
		if(next_message_to_send==12)
		{
			next_message_to_send=0;
		}

		if(next_message_to_send==0)
		{
			// DBT
			if(second_tick_count-depth_receive_time<5)
			{
				sprintf(string_buffer1, "$IIDBT,,,%d.%d,M,,*", (int)depth, (int)((depth*10.0)-((unsigned int)depth)*10));			
				checksum=calculate_checksum(string_buffer1);				
				sprintf(string_buffer2, "%s%02X\r\n", string_buffer1, checksum);
				mUSBUSARTTxRam((byte *)string_buffer2, strlen(string_buffer2));
			}
		}
		else if(next_message_to_send==1)
		{
			// VWR
			if(second_tick_count-awa_receive_time<5 && second_tick_count-aws_receive_time<5)
			{	
				char nmea_awa_side;
				int nmea_awa;

				if(awa<180.0)
				{
					nmea_awa=(int)awa;
					nmea_awa_side='R';
				}	
				else
				{
					nmea_awa=360-(int)awa;
					nmea_awa_side='L';
				}	
				sprintf(string_buffer1, "$IIVWR,%d,%c,%d.%d,N,,,,*", nmea_awa, nmea_awa_side, (int)aws, (int)((aws*10.0)-((unsigned int)aws)*10));			
				checksum=calculate_checksum(string_buffer1);				
				sprintf(string_buffer2, "%s%02X\r\n", string_buffer1, checksum);
				mUSBUSARTTxRam((byte *)string_buffer2, strlen(string_buffer2));
			}
		}
		else if(next_message_to_send==2)
		{
			// VHW
			int true_heading;
			
			true_heading=heading+variation;
			if(true_heading<0)
			{
				true_heading+=360;
			}
			else if(true_heading>=360)
			{
				true_heading-=360;
			}

			if(second_tick_count-heading_receive_time<5 && 
			     second_tick_count-boatspeed_receive_time<5 &&
			     second_tick_count-variation_receive_time<15)
			{	
				// got boatspeed, mag var and heading
				sprintf(string_buffer1, "$IIVHW,%d,T,%d,M,%d.%d,N*", true_heading, heading, (int)boatspeed, (int)((boatspeed*10.0)-((unsigned int)boatspeed)*10));	
				checksum=calculate_checksum(string_buffer1);				
				sprintf(string_buffer2, "%s%02X\r\n", string_buffer1, checksum);		
				mUSBUSARTTxRam((byte *)string_buffer2, strlen(string_buffer2));
			}
			else if(second_tick_count-heading_receive_time<5 && 
			     		second_tick_count-variation_receive_time<15)
			{	
				// got mag var and heading
				sprintf(string_buffer1, "$IIVHW,%d,T,%d,M,,*", true_heading, heading);	
				checksum=calculate_checksum(string_buffer1);				
				sprintf(string_buffer2, "%s%02X\r\n", string_buffer1, checksum);
				mUSBUSARTTxRam((byte *)string_buffer2, strlen(string_buffer2));
			}
			else if(second_tick_count-heading_receive_time<5 && 
			     		second_tick_count-boatspeed_receive_time<5)
			{	
				// got boatspeed and heading
				sprintf(string_buffer1, "$IIVHW,,,%d,M,%d.%d,N*", heading, (int)boatspeed, (int)((boatspeed*10.0)-((unsigned int)boatspeed)*10));	
				checksum=calculate_checksum(string_buffer1);				
				sprintf(string_buffer2, "%s%02X\r\n", string_buffer1, checksum);	
				mUSBUSARTTxRam((byte *)string_buffer2, strlen(string_buffer2));
			}
			else if(second_tick_count-heading_receive_time<5)
			{	
				// heading only
				sprintf(string_buffer1, "$IIVHW,,,%d,M,,N*", heading);	
				checksum=calculate_checksum(string_buffer1);				
				sprintf(string_buffer2, "%s%02X\r\n", string_buffer1, checksum);	
				mUSBUSARTTxRam((byte *)string_buffer2, strlen(string_buffer2));
			}
			else if(second_tick_count-boatspeed_receive_time<5)
			{	
				// got boatspeed only
				sprintf(string_buffer1, "$IIVHW,,,,,%d.%d,N*", (int)boatspeed, (int)((boatspeed*10.0)-((unsigned int)boatspeed)*10));			
				checksum=calculate_checksum(string_buffer1);				
				sprintf(string_buffer2, "%s%02X\r\n", string_buffer1, checksum);
				mUSBUSARTTxRam((byte *)string_buffer2, strlen(string_buffer2));
			}
		}
		else if(next_message_to_send==3)
		{
			// HDM
			if(second_tick_count-heading_receive_time<5)
			{
				sprintf(string_buffer1, "$IIHDM,%d.0,M*", heading);			
				checksum=calculate_checksum(string_buffer1);				
				sprintf(string_buffer2, "%s%02X\r\n", string_buffer1, checksum);
				mUSBUSARTTxRam((byte *)string_buffer2, strlen(string_buffer2));
			}
		}
		else if(next_message_to_send==4)
		{
			// MTW
			if(second_tick_count-temperature_receive_time<30)
			{
				sprintf(string_buffer1, "$IIMTW,%d.%d,C*", (int)temperature, (int)((temperature*10.0)-((unsigned int)temperature)*10));			
				checksum=calculate_checksum(string_buffer1);				
				sprintf(string_buffer2, "%s%02X\r\n", string_buffer1, checksum);
				mUSBUSARTTxRam((byte *)string_buffer2, strlen(string_buffer2));
			}
		}
		else if(next_message_to_send==5)
		{
			char lat_hemi='N';
			char long_hemi='E';
			float nmea_lat;
			float nmea_long;

			nmea_lat=fabs((float)latitude_degrees*100.0f)+latitude_minutes;
			if(latitude_degrees<0)
			{
				lat_hemi='S';
			}	
			nmea_long=fabs((float)longitude_degrees*100.0f)+longitude_minutes;
			if(longitude_degrees<0)
			{
				long_hemi='W';
			}	

			// RMC
			if(second_tick_count-sog_receive_time<5 && 
			     second_tick_count-cog_receive_time<5 &&
			     second_tick_count-latitude_receive_time<5 &&
			     second_tick_count-longitude_receive_time<5 &&
			     second_tick_count-variation_receive_time<15)
			{	
				if(mUSBUSARTIsTxTrfReady())
				{		
					sprintf(string_buffer1, "$IIRMC,%02d%02d%02d,A,%04d.%02d,%c,%05d.%02d,%c,%d.%d,%d,%02d%02d%02d,%d,%c*", 
											hour, minute, second,
					                        (int)nmea_lat,
					                        (int)((nmea_lat*100.0f)-((unsigned long)nmea_lat)*100),
					                        lat_hemi,
					                        (int)nmea_long,
					                        (int)((nmea_long*100.0f)-((unsigned long)nmea_long)*100),
					                        long_hemi,
					                        (int)sog, 
					                        (int)((sog*10.0f)-((unsigned int)sog)*10), 
					                        cog,
											date, month, year,
											abs(variation),
											variation<0 ? 'W' : 'E');			
					checksum=calculate_checksum(string_buffer1);				
					sprintf(string_buffer2, "%s%02X\r\n", string_buffer1, checksum);
					mUSBUSARTTxRam((byte *)string_buffer2, strlen(string_buffer2));
				}
			}
			else if(second_tick_count-sog_receive_time<5 && 
			     second_tick_count-cog_receive_time<5 &&
			     second_tick_count-latitude_receive_time<5 &&
			     second_tick_count-longitude_receive_time<5)
			{			
				if(mUSBUSARTIsTxTrfReady())
				{				
					sprintf(string_buffer1, "$IIRMC,%02d%02d%02d,A,%04d.%02d,%c,%05d.%02d,%c,%d.%d,%d,%02d%02d%02d,,*", 
											hour, minute, second,
					                        (int)nmea_lat,
					                        (int)((nmea_lat*100.0f)-((unsigned long)nmea_lat)*100),
					                        lat_hemi,
					                        (int)nmea_long,
					                        (int)((nmea_long*100.0f)-((unsigned long)nmea_long)*100),
					                        long_hemi,
					                        (int)sog, 
					                        (int)((sog*10.0f)-((unsigned int)sog)*10), 
					                        cog,
											date, month, year);			
					checksum=calculate_checksum(string_buffer1);				
					sprintf(string_buffer2, "%s%02X\r\n", string_buffer1, checksum);
					mUSBUSARTTxRam((byte *)string_buffer2, strlen(string_buffer2));
				}
			}
		}
		else if(next_message_to_send==6)
		{
			// VLW
			if(second_tick_count-triplog_receive_time<10 || (second_tick_count-log_receive_time<10 && second_tick_count-trip_receive_time<10))
			{
				// trip and log
				sprintf(string_buffer1, "$IIVLW,%ld.%ld,N,%ld.%ld,N*", (long)boat_log, (long)((boat_log*10.0)-((unsigned long)boat_log)*10),
					(long)trip, (long)((trip*10.0)-((unsigned long)trip)*10));
				checksum=calculate_checksum(string_buffer1);				
				sprintf(string_buffer2, "%s%02X\r\n", string_buffer1, checksum);
				mUSBUSARTTxRam((byte *)string_buffer2, strlen(string_buffer2));
			}
			else if(second_tick_count-log_receive_time<10)
			{
				// log only
				sprintf(string_buffer1, "$IIVLW,%ld.%ld,N,,*", (long)boat_log, (long)((boat_log*10.0)-((unsigned long)boat_log)*10));
				checksum=calculate_checksum(string_buffer1);				
				sprintf(string_buffer2, "%s%02X\r\n", string_buffer1, checksum);
				mUSBUSARTTxRam((byte *)string_buffer2, strlen(string_buffer2));
			}
			else if(second_tick_count-trip_receive_time<10)
			{
				// trip only
				sprintf(string_buffer1, "$IIVLW,,,%ld.%ld,N*", (long)trip, (long)((trip*10.0)-((unsigned long)trip)*10));
				checksum=calculate_checksum(string_buffer1);				
				sprintf(string_buffer2, "%s%02X\r\n", string_buffer1, checksum);
				mUSBUSARTTxRam((byte *)string_buffer2, strlen(string_buffer2));
			}
		}
		else if(next_message_to_send==7)
		{
			// RMB
			int bearing_to_destination_true=bearing_to_destination+variation;
			if(bearing_to_destination_true<0)
			{	
				bearing_to_destination_true+=360;
			}
			else if(bearing_to_destination_true>=360)
			{	
				bearing_to_destination_true-=360;
			}

			if(second_tick_count-nav_to_waypoint_receive_time<10)
			{
				sprintf(string_buffer1, "$IIRMB,A,,,,,,,,,%ld.%ld,%d.0,,*", 
					(long)distance_to_destination, (long)((distance_to_destination*10.0)-((unsigned long)distance_to_destination)*10),
					bearing_to_destination_true);
				checksum=calculate_checksum(string_buffer1);				
				sprintf(string_buffer2, "%s%02X\r\n", string_buffer1, checksum);
				mUSBUSARTTxRam((byte *)string_buffer2, strlen(string_buffer2));
			}
		}
		else if(next_message_to_send==8)
		{
			// MWV - true
			if(second_tick_count-true_wind_receive_time<10)
			{

				sprintf(string_buffer1, "$IIMWV,%d,T,%d.%d,N,A*",(int)twa, (int)tws, (int)((tws*10.0)-((unsigned int)tws)*10));
				checksum=calculate_checksum(string_buffer1);				
				sprintf(string_buffer2, "%s%02X\r\n", string_buffer1, checksum);
				mUSBUSARTTxRam((byte *)string_buffer2, strlen(string_buffer2));
			}
		}
		else if(next_message_to_send==9)
		{
			// MWV - apparent
			if(second_tick_count-awa_receive_time<5 && second_tick_count-aws_receive_time<5)
			{
				sprintf(string_buffer1, "$IIMWV,%d,R,%d.%d,N,A*",(int)awa, (int)aws, (int)((aws*10.0)-((unsigned int)aws)*10));
				checksum=calculate_checksum(string_buffer1);				
				sprintf(string_buffer2, "%s%02X\r\n", string_buffer1, checksum);
				mUSBUSARTTxRam((byte *)string_buffer2, strlen(string_buffer2));
			}
		}
		else if(next_message_to_send==10)
		{
			// VWT
			if(second_tick_count-true_wind_receive_time<5)
			{	
				char nmea_twa_side;
				int nmea_twa;

				if(twa<180.0)
				{
					nmea_twa=(int)twa;
					nmea_twa_side='R';
				}	
				else
				{
					nmea_twa=360-(int)twa;
					nmea_twa_side='L';
				}	
				sprintf(string_buffer1, "$IIVWT,%d,%c,%d.%d,N,,,,*", nmea_twa, nmea_twa_side, (int)tws, (int)((tws*10.0)-((unsigned int)tws)*10));			
				checksum=calculate_checksum(string_buffer1);				
				sprintf(string_buffer2, "%s%02X\r\n", string_buffer1, checksum);
				mUSBUSARTTxRam((byte *)string_buffer2, strlen(string_buffer2));
			}
		}
		else if(next_message_to_send==11)
		{
			// RSA
			if(second_tick_count-rudder_receive_time<5)
			{	
				sprintf(string_buffer1, "$IIRSA,%d,A,,*", (unsigned int)rudder);			
				checksum=calculate_checksum(string_buffer1);				
				sprintf(string_buffer2, "%s%02X\r\n", string_buffer1, checksum);
				mUSBUSARTTxRam((byte *)string_buffer2, strlen(string_buffer2));
			}
		}
		next_message_to_send++;
	}
}



