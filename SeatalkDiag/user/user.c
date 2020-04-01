/** I N C L U D E S **********************************************************/
#include <p18cxxx.h>
#include <stdio.h>
#include <string.h>
#include "system\typedefs.h"
#include "system\usb\usb.h"
#include "io_cfg.h"             
#include "user\user.h"
#include "user\seatalk.h"

extern unsigned int flags;
extern float xte;
extern char direction_to_steer;
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
extern float sog;
extern float awa;
extern unsigned int cog;
extern float aws;
extern unsigned int heading;
extern float distance_to_destination;
extern unsigned int bearing_to_destination;
extern unsigned char lamps;
extern unsigned char autopilot_state;
extern char string_buffer1[150];
extern unsigned char keystroke;
extern unsigned char arrival_circle_entered;
extern unsigned char perpendicular_passed;
extern char waypoint_id[4];

void seatalk_message_handler(unsigned char message_type, volatile unsigned char *data)
{

	unsigned char i;
	char hex_buffer[8];

	strcpypgm2ram(string_buffer1, ":");
	if(!mUSBUSARTIsTxTrfReady())
	{
		return;
	}	
	
	switch(message_type)
	{
		case LAMPS:
			sprintf(string_buffer1, "Lamps %d : ", lamps);		
			for(i=0;i< 3+ ((*(data+1)) & 0x0f);i++)
			{
				sprintf(hex_buffer, "0x%02x ", *(data+i));
				strcat(string_buffer1, hex_buffer);
			}			
			strcatpgm2ram(string_buffer1, "\r\n");
			mUSBUSARTTxRam((byte *)string_buffer1, strlen(string_buffer1));
			break;

		case GMT:
			sprintf(string_buffer1, "Time %02d:%02d:%02d : ", hour, minute,second);
			for(i=0;i< 3+ ((*(data+1)) & 0x0f);i++)
			{
				sprintf(hex_buffer, "0x%02x ", *(data+i));
				strcat(string_buffer1, hex_buffer);
			}			
			strcatpgm2ram(string_buffer1, "\r\n");		
			mUSBUSARTTxRam((byte *)string_buffer1, strlen(string_buffer1));
			break;

		case DATE:
			sprintf(string_buffer1, "Date %d/%d/%d : ", date, month, year);		
			for(i=0;i< 3+ ((*(data+1)) & 0x0f);i++)
			{
				sprintf(hex_buffer, "0x%02x ", *(data+i));
				strcat(string_buffer1, hex_buffer);
			}			
			strcatpgm2ram(string_buffer1, "\r\n");
			mUSBUSARTTxRam((byte *)string_buffer1, strlen(string_buffer1));
			break;

		case NAV_TO_WAYPOINT:
			sprintf(string_buffer1, "Dist todest %d.%d Nm Bear to dest %d deg Xte %d.%d Nm Dir to steer %c Flags %01x : ", (int)distance_to_destination, 
					(int)((distance_to_destination*10.0)-((unsigned int)distance_to_destination)*10),
					bearing_to_destination, 
					(int)xte, (int)((xte*10.0)-((unsigned int)xte)*10),
					direction_to_steer, 
					flags);		
			for(i=0;i< 3+ ((*(data+1)) & 0x0f);i++)
			{
				sprintf(hex_buffer, "0x%02x ", *(data+i));
				strcat(string_buffer1, hex_buffer);
			}		
			strcatpgm2ram(string_buffer1, "\r\n");
			mUSBUSARTTxRam((byte *)string_buffer1, strlen(string_buffer1));
			break;

		case TRIPLOG:
			sprintf(string_buffer1, "Trip %d.%d Nm\r\nLog %d.%d Nm : ", (int)trip, (int)((trip*10.0)-((unsigned int)trip)*10),
					(int)boat_log, (int)((boat_log*10.0)-((unsigned int)boat_log)*10));
			for(i=0;i< 3+ ((*(data+1)) & 0x0f);i++)
			{
				sprintf(hex_buffer, "0x%02x ", *(data+i));
				strcat(string_buffer1, hex_buffer);
			}			
			strcatpgm2ram(string_buffer1, "\r\n");	
			mUSBUSARTTxRam((byte *)string_buffer1, strlen(string_buffer1));	
			break;

		case TEMPERATURE:
			sprintf(string_buffer1, "Temp %d.%d C : ", (int)temperature, (int)((temperature*10.0)-((unsigned int)temperature)*10));		
			for(i=0;i< 3+ ((*(data+1)) & 0x0f);i++)
			{
				sprintf(hex_buffer, "0x%02x ", *(data+i));
				strcat(string_buffer1, hex_buffer);
			}			
			strcatpgm2ram(string_buffer1, "\r\n");
			mUSBUSARTTxRam((byte *)string_buffer1, strlen(string_buffer1));
			break;

		case VARIATION:
			sprintf(string_buffer1, "Variation %d degrees : ", variation);		
			for(i=0;i< 3+ ((*(data+1)) & 0x0f);i++)
			{
				sprintf(hex_buffer, "0x%02x ", *(data+i));
				strcat(string_buffer1, hex_buffer);
			}			
			strcatpgm2ram(string_buffer1, "\r\n");
			mUSBUSARTTxRam((byte *)string_buffer1, strlen(string_buffer1));	
			break;

		case DEPTH:
			sprintf(string_buffer1, "Depth %d.%d m : ", (int)depth, (int)((depth*10.0)-((unsigned int)depth)*10));
			for(i=0;i< 3+ ((*(data+1)) & 0x0f);i++)
			{
				sprintf(hex_buffer, "0x%02x ", *(data+i));
				strcat(string_buffer1, hex_buffer);
			}			
			strcatpgm2ram(string_buffer1, "\r\n");		
			mUSBUSARTTxRam((byte *)string_buffer1, strlen(string_buffer1));	
			break;

		case SOG:
			sprintf(string_buffer1, "SOG %d.%d knts : ", (int)sog, (int)((sog*10.0)-((unsigned int)sog)*10));
			for(i=0;i< 3+ ((*(data+1)) & 0x0f);i++)
			{
				sprintf(hex_buffer, "0x%02x ", *(data+i));
				strcat(string_buffer1, hex_buffer);
			}			
			strcatpgm2ram(string_buffer1, "\r\n");		
			mUSBUSARTTxRam((byte *)string_buffer1, strlen(string_buffer1));	
			break;
			
		case COG:
			sprintf(string_buffer1, "COG %d deg : ", cog);	
			for(i=0;i< 3+ ((*(data+1)) & 0x0f);i++)
			{
				sprintf(hex_buffer, "0x%02x ", *(data+i));
				strcat(string_buffer1, hex_buffer);
			}			
			strcatpgm2ram(string_buffer1, "\r\n");	
			mUSBUSARTTxRam((byte *)string_buffer1, strlen(string_buffer1));	
			break;
			
		case LATITUDE:
			sprintf(string_buffer1, "Latitude %d %d.%02d : ", latitude_degrees, (int)latitude_minutes, 
					(int)((latitude_minutes*100.0f)-((unsigned long)latitude_minutes)*100));
			for(i=0;i< 3+ ((*(data+1)) & 0x0f);i++)
			{
				sprintf(hex_buffer, "0x%02x ", *(data+i));
				strcat(string_buffer1, hex_buffer);
			}			
			strcatpgm2ram(string_buffer1, "\r\n");		
			mUSBUSARTTxRam((byte *)string_buffer1, strlen(string_buffer1));	
			break;
			
		case LONGITUDE:
			sprintf(string_buffer1, "Longitude %d %d.%02d : ", longitude_degrees, (int)longitude_minutes, 
					(int)((longitude_minutes*100.0f)-((unsigned long)longitude_minutes)*100));	
			for(i=0;i< 3+ ((*(data+1)) & 0x0f);i++)
			{
				sprintf(hex_buffer, "0x%02x ", *(data+i));
				strcat(string_buffer1, hex_buffer);
			}			
			strcatpgm2ram(string_buffer1, "\r\n");	
			mUSBUSARTTxRam((byte *)string_buffer1, strlen(string_buffer1));	
			break;
			
		case APPARENT_WIND_ANGLE:
			sprintf(string_buffer1, "Apparent Wind Angle %d.%d deg : ", (int)awa, (int)((awa*10.0)-((unsigned int)awa)*10));		
			for(i=0;i< 3+ ((*(data+1)) & 0x0f);i++)
			{
				sprintf(hex_buffer, "0x%02x ", *(data+i));
				strcat(string_buffer1, hex_buffer);
			}			
			strcatpgm2ram(string_buffer1, "\r\n");		
			mUSBUSARTTxRam((byte *)string_buffer1, strlen(string_buffer1));	
			break;
			
		case APPARENT_WIND_SPEED:
			sprintf(string_buffer1, "Apparent Wind Speed %d.%d knts : ", (int)aws, (int)((aws*10.0)-((unsigned int)aws)*10));		
			for(i=0;i< 3+ ((*(data+1)) & 0x0f);i++)
			{
				sprintf(hex_buffer, "0x%02x ", *(data+i));
				strcat(string_buffer1, hex_buffer);
			}			
			strcatpgm2ram(string_buffer1, "\r\n");
			mUSBUSARTTxRam((byte *)string_buffer1, strlen(string_buffer1));	
			break;

		case HEADING1:
			{
				unsigned char c='?';

				if(autopilot_state==APS_STANDBY)
				{
					c='S';			
				}
				else if(autopilot_state==APS_AUTO)
				{
					c='A';			
				}

				sprintf(string_buffer1, "Heading %d deg magnetic Auto=%c : ", heading, c);	
			for(i=0;i< 3+ ((*(data+1)) & 0x0f);i++)
			{
				sprintf(hex_buffer, "0x%02x ", *(data+i));
				strcat(string_buffer1, hex_buffer);
			}			
			strcatpgm2ram(string_buffer1, "\r\n");	
				mUSBUSARTTxRam((byte *)string_buffer1, strlen(string_buffer1));	
			}
			break;

		case BOATSPEED:
			sprintf(string_buffer1, "Boatspeed %d.%d Knts : ", (int)boatspeed, (int)((boatspeed*10.0)-((unsigned int)boatspeed)*10));			
			for(i=0;i< 3+ ((*(data+1)) & 0x0f);i++)
			{
				sprintf(hex_buffer, "0x%02x ", *(data+i));
				strcat(string_buffer1, hex_buffer);
			}			
			strcatpgm2ram(string_buffer1, "\r\n");
			mUSBUSARTTxRam((byte *)string_buffer1, strlen(string_buffer1));	
			break;		

		case ARRIVAL_INFO:
			sprintf(string_buffer1, "Perp passed %u Circle entered %u Waypoint id %c%c%c%c : ", 
				perpendicular_passed, arrival_circle_entered, waypoint_id[0], waypoint_id[1], waypoint_id[2], waypoint_id[3]);			
			for(i=0;i< 3+ ((*(data+1)) & 0x0f);i++)
			{
				sprintf(hex_buffer, "0x%02x ", *(data+i));
				strcat(string_buffer1, hex_buffer);
			}			
			strcatpgm2ram(string_buffer1, "\r\n");
			mUSBUSARTTxRam((byte *)string_buffer1, strlen(string_buffer1));	
			break;
			
		case WAYPOINT_ID:
			sprintf(string_buffer1, "Waypoint id %c%c%c%c : ", waypoint_id[0], waypoint_id[1], waypoint_id[2], waypoint_id[3]);

			for(i=0;i< 3+ ((*(data+1)) & 0x0f);i++)
			{
				sprintf(hex_buffer, "0x%02x ", *(data+i));
				strcat(string_buffer1, hex_buffer);
			}			
			strcatpgm2ram(string_buffer1, "\r\n");			mUSBUSARTTxRam((byte *)string_buffer1, strlen(string_buffer1));	
			break;
			
		case KEYSTROKE:
			sprintf(string_buffer1, "Keystroke ");	
			switch(keystroke)
			{
				case KS_AUTO:
					strcatpgm2ram(string_buffer1, "A");			
					break;

				case KS_STANDBY:
					strcatpgm2ram(string_buffer1, "S");			
					break;

				case KS_MINUS_1:
					strcatpgm2ram(string_buffer1, "-1");			
					break;
					
				case KS_PLUS_1:
					strcatpgm2ram(string_buffer1, "+1");							
					break;
					
				case KS_MINUS_10:
					strcatpgm2ram(string_buffer1, "-10");							
					break;
					
				case KS_PLUS_10:
					strcatpgm2ram(string_buffer1, "+10");							
					break;
			}	
			strcatpgm2ram(string_buffer1, " : ");
			for(i=0;i< 3+ ((*(data+1)) & 0x0f);i++)
			{
				sprintf(hex_buffer, "0x%02x ", *(data+i));
				strcat(string_buffer1, hex_buffer);
			}			
			strcatpgm2ram(string_buffer1, "\r\n");		
			mUSBUSARTTxRam((byte *)string_buffer1, strlen(string_buffer1));	
			break;		

		default:
			sprintf(string_buffer1, "UNHANDLED : ");		
			for(i=0;i< 3+ ((*(data+1)) & 0x0f);i++)
			{
				sprintf(hex_buffer, "0x%02x ", *(data+i));
				strcat(string_buffer1, hex_buffer);
			}			
			strcatpgm2ram(string_buffer1, "\r\n");
			mUSBUSARTTxRam((byte *)string_buffer1, strlen(string_buffer1));
			break;
	}
}

void UserInit(void)
{   
	init_seatalk(seatalk_message_handler);
}

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
 * Overview:        This function is a place holder for other user routines.
 *                  It is a mixture of both USB and non-USB tasks.
 *
 * Note:            None
 *****************************************************************************/
void ProcessIO(void)
{
    if((usb_device_state < CONFIGURED_STATE)||(UCONbits.SUSPND==1)) 
	{
		return; 
	}

	parse_next_seatalk_message();
}



