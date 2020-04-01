#include <p18cxxx.h>
#include <string.h>
#include <limits.h>
#include <ctype.h>
#include <float.h>
#include <math.h>
#include "nmea.h"
#include "st2nmea.h"
#include "util.h"
#include "sscanf.h"

#ifdef _DEBUG
extern char debug_string[];
#endif

extern volatile char nmea_messages_in[NUMBER_NMEA_MESSAGES][NMEA_MAX_MESSAGE_LENGTH+1];
extern char next_message_field[NMEA_MAX_MESSAGE_LENGTH+1];
extern volatile unsigned int nmea_out_space;
extern volatile unsigned char nmea_messages_out[NMEA_OUT_BUFFER_SIZE];
extern volatile unsigned int nmea_out_next_write_pos;
extern volatile unsigned long millisecond_tick_count;
extern char nmea_message[NMEA_MAX_MESSAGE_LENGTH+1];

// nmea add more data variables here
extern float nmea_depth;
extern unsigned long nmea_depth_receive_time;
extern float nmea_boatspeed;
extern unsigned long nmea_boatspeed_receive_time;
extern float nmea_heading_magnetic;
extern unsigned long nmea_heading_magnetic_receive_time;
extern float nmea_heading_true;
extern unsigned long nmea_heading_true_receive_time;
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
extern float nmea_xte;
extern unsigned long nmea_xte_receive_time;
extern char nmea_direction_to_steer;
extern unsigned long nmea_direction_to_steer_receive_time;
extern char nmea_destination_waypoint_name[];
extern unsigned long nmea_destination_waypoint_name_receive_time;
extern float nmea_distance_to_destination;
extern unsigned long nmea_distance_to_destination_receive_time;
extern float nmea_bearing_to_destination;
extern unsigned long nmea_bearing_to_destination_receive_time;
extern unsigned char nmea_waypoint_arrived;
extern unsigned long nmea_waypoint_arrived_receive_time;
static unsigned char nmea_calculate_checksum(char *message);
static void nmea_add_checksum_and_send(char *message);
static void nmea_add_header(nmea_message_type_t nmea_message_type);
static void nmea_add_time(time_t time);
static void nmea_add_latitude(nav_data_t *nav_data);
static void nmea_add_longitude(nav_data_t *nav_data);

// add new message types here
static const rom char *nmea_headers="DPT" "DBT" "VHW" "RSA" "HDM" "HDG" "HDT" "MTW" "VLW" "VWR" "VWT" "MWV" "RMC" "GLL";

static void nmea_basic_float_parse(unsigned char message_number, unsigned char field, float *result, unsigned long *time);
static void nmea_add_time(time_t time);
static void nmea_add_latitude(nav_data_t *nav_data);
static void nmea_add_longitude(nav_data_t *nav_data);
static void nmea_add_basic_float(float value);

nmea_message_type_t nmea_identify_message_type(char *message)
{
	unsigned char i;
	const rom char *next_header=nmea_headers;
	
	for(i=0; i<NMEA_UNKNOWN; i++)
	{
		if(*next_header==*message && *(next_header+1)==*(message+1) && *(next_header+2)==*(message+2))
		{
			return i;
		}	
		next_header+=3;
	}
	
	return NMEA_UNKNOWN;	
}

char *nmea_get_name_from_type(nmea_message_type_t message_type)
{
	static char name[4];
	unsigned char header_pos=message_type*3;
	
	if(message_type<NMEA_UNKNOWN)
	{
		memcpypgm2ram(name, nmea_headers+3*message_type, 3);
		name[3]='\0';
	}
	else
	{
		strcpypgm2ram(name, "???");
	}		

	return name;
}

static unsigned char nmea_calculate_checksum(char *message)
{
	unsigned char checksum=0;
	unsigned char length;
	unsigned char i;

	length=strlen(message)-1;
	for(i=0; i<length; i++)
	{
		checksum^=message[i+1];
	}
	
	return checksum;
}

static void nmea_add_checksum_and_send(char *message)
{
	char checksum_text[3];
	unsigned char checksum=nmea_calculate_checksum(message);	

	strcatpgm2ram(message, "*");		
	strcat(message, util_btoh(checksum));	
	strcatpgm2ram(message, "\r\n");		
	nmea_queue_message_to_send(message);
}

void nmea_queue_message_to_send(char *message)
{
	unsigned char next=0;

	if(nmea_out_space<strlen(message))
	{
		return;
	}

	while(message[next]!='\0')
	{
		nmea_messages_out[nmea_out_next_write_pos]=message[next];
		next++;
		nmea_out_space--;
		nmea_out_next_write_pos++;
		if(nmea_out_next_write_pos==NMEA_OUT_BUFFER_SIZE)
		{
			nmea_out_next_write_pos=0;
		}
	}

	PIE1bits.TX1IE=1;
}

static void nmea_basic_float_parse(unsigned char message_number, unsigned char field, float *result, unsigned long *time)
{
	if(util_get_field(field, (char *)(nmea_messages_in[message_number]+1), next_message_field, ',')>0)
	{
		sscanf(next_message_field, "%f", result);
		*time=millisecond_tick_count;
	}
}	

void nmea_process_next_message(void)
{
	static unsigned char message_position=0;
	nmea_message_type_t message_type;
	float true_wind_speed;
	float true_wind_angle;
	float units_factor;
	float coord;

    // check if next message has been fully received and ready for processing
	if(nmea_messages_in[message_position][0]==MS_READY)
	{
	    // set message state to reading to stop it being overwritten
		nmea_messages_in[message_position][0]=MS_READING;
		
		if(*(nmea_messages_in[message_position]+1)=='#')
		{
			// config message
			util_decode_setting_message((char *)(nmea_messages_in[message_position]+2));
		}
		else
		{
			// normal nmea message
			message_type=nmea_identify_message_type((char *)(nmea_messages_in[message_position]+4));

			switch(message_type)
			{
				case NMEA_DPT:
					nmea_basic_float_parse(message_position, 1, &nmea_depth, &nmea_depth_receive_time);
					break;

				case NMEA_DBT:
					nmea_basic_float_parse(message_position, 3, &nmea_depth, &nmea_depth_receive_time);
					break;

				case NMEA_VHW:
					nmea_basic_float_parse(message_position, 1, &nmea_heading_true, &nmea_heading_true_receive_time);
					nmea_basic_float_parse(message_position, 3, &nmea_heading_magnetic, &nmea_heading_magnetic_receive_time);					
					nmea_basic_float_parse(message_position, 5, &nmea_boatspeed, &nmea_boatspeed_receive_time);									
					break;

				case NMEA_RSA:
					if(util_get_field(1, (char *)(nmea_messages_in[message_position]+1), next_message_field, ',')>0)
					{
						sscanf(next_message_field, "%f", &nmea_rudder);
						if(util_get_field(2, (char *)(nmea_messages_in[message_position]+1), next_message_field, ',')==1)
						{
							if(next_message_field[0]=='A')
							{
								nmea_rudder_receive_time=millisecond_tick_count;
							}
						}
					}
					break;

				case NMEA_HDM:
					nmea_basic_float_parse(message_position, 1, &nmea_heading_magnetic, &nmea_heading_magnetic_receive_time);												
					break;

				case NMEA_HDG:
					nmea_basic_float_parse(message_position, 1, &nmea_heading_magnetic, &nmea_heading_magnetic_receive_time);												

					if(util_get_field(4, (char *)(nmea_messages_in[message_position]+1), next_message_field, ',')>0)
					{
						sscanf(next_message_field, "%f", &nmea_variation);

						if(util_get_field(5, (char *)(nmea_messages_in[message_position]+1), next_message_field, ',')==1)
						{
							if(*next_message_field=='W')
							{
								nmea_variation=-nmea_variation;
							}
							nmea_variation_receive_time=millisecond_tick_count;
						}
					}
					break;

				case NMEA_HDT:
					// give up if not received variation data recently
					if(millisecond_tick_count-nmea_variation_receive_time>MAX_DATA_AGE_MS)
					{
						nmea_basic_float_parse(message_position, 1, &nmea_heading_magnetic, &nmea_heading_magnetic_receive_time);
						nmea_heading_magnetic-=nmea_variation;
						if(nmea_heading_magnetic<0.0f)
						{
							nmea_heading_magnetic+=360.0f;
						}	
						else if(nmea_heading_magnetic>360.0f)
						{
							nmea_heading_magnetic-=360.0f;
						}	
					}	
					break;
					
				case NMEA_MTW:
					nmea_basic_float_parse(message_position, 1, &nmea_temperature, &nmea_temperature_receive_time);
					break;
					
				case NMEA_VLW:
					nmea_basic_float_parse(message_position, 1, &nmea_log, &nmea_log_receive_time);
					nmea_basic_float_parse(message_position, 3, &nmea_trip, &nmea_trip_receive_time);
					break;
					
				case NMEA_VWR:
					if(util_get_field(1, (char *)(nmea_messages_in[message_position]+1), next_message_field, ',')>0)
					{
						sscanf(next_message_field, "%f", &nmea_apparent_wind_angle);
						util_get_field(2, (char *)(nmea_messages_in[message_position]+1), next_message_field, ',');
						if(*next_message_field=='L' || *next_message_field=='R')
						{
							if(*next_message_field=='L')
							{
								nmea_apparent_wind_angle=360.0f-nmea_apparent_wind_angle;
							}	
							nmea_apparent_wind_angle_receive_time=millisecond_tick_count;
						}
					}	
					nmea_basic_float_parse(message_position, 3, &nmea_apparent_wind_speed, &nmea_apparent_wind_speed_receive_time);
					break;				
					
				case NMEA_VWT:
					// give up if received apparent wind data recently
					if(millisecond_tick_count-nmea_apparent_wind_angle_receive_time<MAX_DATA_AGE_MS &&
						millisecond_tick_count-nmea_apparent_wind_speed_receive_time<MAX_DATA_AGE_MS)
					{
						break;
					}		
					
					// give up if not received boat speed data recently
					if(millisecond_tick_count-nmea_boatspeed_receive_time>MAX_DATA_AGE_MS)
					{
						break;
					}
					
					if(util_get_field(1, (char *)(nmea_messages_in[message_position]+1), next_message_field, ',')>0)
					{
						sscanf(next_message_field, "%f", &true_wind_angle);
						util_get_field(2, (char *)(nmea_messages_in[message_position]+1), next_message_field, ',');
						if(*next_message_field=='L' || *next_message_field=='R')
						{
							if(*next_message_field=='L')
							{
								true_wind_angle=360.0f-true_wind_angle;
							}	
								
							if(util_get_field(3, (char *)(nmea_messages_in[message_position]+1), next_message_field, ',')>0)
							{
								sscanf(next_message_field, "%f", &true_wind_speed);
								util_calc_apparent_wind_from_true(true_wind_speed, true_wind_angle, nmea_boatspeed, &nmea_apparent_wind_speed, &nmea_apparent_wind_angle);
								nmea_apparent_wind_speed_receive_time=millisecond_tick_count;
								nmea_apparent_wind_angle_receive_time=millisecond_tick_count;
							}							
						}	
					}	
					break;
					
				case NMEA_MWV:
					util_get_field(5, (char *)(nmea_messages_in[message_position]+1), next_message_field, ',');
					if(*next_message_field!='A')
					{
						break;
					}
					
					util_get_field(4, (char *)(nmea_messages_in[message_position]+1), next_message_field, ',');
					switch(*next_message_field)
					{
						case 'N':
							units_factor=1.0f;
							break;
							
						case 'M':
							units_factor=KNOTS_IN_MPS;		
							break;
							
						case 'K':
							units_factor=KNOTS_IN_KMPH;		
							break;
							
						default:
							units_factor=FLT_MAX;
					}
					if(units_factor==FLT_MAX)
					{
						break;
					}
						
					util_get_field(2, (char *)(nmea_messages_in[message_position]+1), next_message_field, ',');
					if(*next_message_field=='T')
					{
						// give up if received apparent data recently
						if(millisecond_tick_count-nmea_apparent_wind_angle_receive_time<MAX_DATA_AGE_MS &&
							millisecond_tick_count-nmea_apparent_wind_speed_receive_time<MAX_DATA_AGE_MS)
						{
							break;
						}		
						
						// give up if not received boat speed data recently
						if(millisecond_tick_count-nmea_boatspeed_receive_time>MAX_DATA_AGE_MS)
						{
							break;
						}
						
						if(util_get_field(1, (char *)(nmea_messages_in[message_position]+1), next_message_field, ',')>0)
						{
							sscanf(next_message_field, "%f", &true_wind_angle);
							if(util_get_field(3, (char *)(nmea_messages_in[message_position]+1), next_message_field, ',')>0)
							{
								sscanf(next_message_field, "%f", &true_wind_speed);
								true_wind_speed*=units_factor;
								util_calc_apparent_wind_from_true(true_wind_speed, true_wind_angle, nmea_boatspeed, &nmea_apparent_wind_speed, &nmea_apparent_wind_angle);
								nmea_apparent_wind_speed_receive_time=millisecond_tick_count;
								nmea_apparent_wind_angle_receive_time=millisecond_tick_count;
							}
						}
					}
					else if(*next_message_field=='R')
					{
						nmea_basic_float_parse(message_position, 1, &nmea_apparent_wind_angle, &nmea_apparent_wind_angle_receive_time);
						
						if(util_get_field(3, (char *)(nmea_messages_in[message_position]+1), next_message_field, ',')>0)
						{
							sscanf(next_message_field, "%f", &nmea_apparent_wind_speed);
							nmea_apparent_wind_speed*=units_factor;
							nmea_apparent_wind_speed_receive_time=millisecond_tick_count;
						}
					}						
					break;
					
				case NMEA_RMC:
					util_get_field(2, (char *)(nmea_messages_in[message_position]+1), next_message_field, ',');
					if(*next_message_field!='A')
					{
						break;
					}	
					
					if(util_get_field(1, (char *)(nmea_messages_in[message_position]+1), next_message_field, ',')>=6)
					{
						nmea_time.hour=(next_message_field[0]-'0')*10+(next_message_field[1]-'0');
						nmea_time.minute=(next_message_field[2]-'0')*10+(next_message_field[3]-'0');
						nmea_time.second=(next_message_field[4]-'0')*10+(next_message_field[5]-'0');
						if(nmea_time.hour<24 && nmea_time.minute<60 && nmea_time.second<60)
						{
							nmea_time_receive_time=millisecond_tick_count;
						}	
					}	
		
					if(util_get_field(3, (char *)(nmea_messages_in[message_position]+1), next_message_field, ',')>0)
					{
						sscanf(next_message_field, "%f", &coord);
						nmea_latitude_degrees=(signed int)(coord/100.0f);
						nmea_latitude_minutes=100.0f*frac(coord/100.0f);
						
						util_get_field(4, (char *)(nmea_messages_in[message_position]+1), next_message_field, ',');
						if(*next_message_field=='N' || *next_message_field=='S')
						{
							if(*next_message_field=='S')
							{
								nmea_latitude_degrees=-nmea_latitude_degrees;
								nmea_latitude_minutes=-nmea_latitude_minutes;
							}
							nmea_latitude_receive_time=millisecond_tick_count;
						}	
					}
					
					if(util_get_field(5, (char *)(nmea_messages_in[message_position]+1), next_message_field, ',')>0)
					{
						sscanf(next_message_field, "%f", &coord);
						nmea_longitude_degrees=(signed int)(coord/100.0f);
						nmea_longitude_minutes=100.0f*frac(coord/100.0f);
						
						util_get_field(6, (char *)(nmea_messages_in[message_position]+1), next_message_field, ',');
						if(*next_message_field=='E' || *next_message_field=='W')
						{
							if(*next_message_field=='W')
							{
								nmea_longitude_degrees=-nmea_longitude_degrees;
								nmea_longitude_minutes=-nmea_longitude_minutes;
							}	
							nmea_longitude_receive_time=millisecond_tick_count;
						}	
					}
						
					nmea_basic_float_parse(message_position, 7, &nmea_sog, &nmea_sog_receive_time);
					nmea_basic_float_parse(message_position, 8, &nmea_cog, &nmea_cog_receive_time);							
		
					if(util_get_field(9, (char *)(nmea_messages_in[message_position]+1), next_message_field, ',')==6)
					{
						nmea_date.date=(next_message_field[0]-'0')*10+(next_message_field[1]-'0');
						nmea_date.month=(next_message_field[2]-'0')*10+(next_message_field[3]-'0');
						nmea_date.year=(next_message_field[4]-'0')*10+(next_message_field[5]-'0');
						if(nmea_date.month<13 && nmea_date.date<32)
						{
							nmea_date_receive_time=millisecond_tick_count;
						}	
					}
					
					if(util_get_field(10, (char *)(nmea_messages_in[message_position]+1), next_message_field, ',')>0)
					{
						sscanf(next_message_field, "%f", &nmea_variation);
						util_get_field(11, (char *)(nmea_messages_in[message_position]+1), next_message_field, ',');
						if(*next_message_field=='E' || *next_message_field=='W')
						{
							if(*next_message_field=='W')
							{
								nmea_variation=-nmea_variation;
							}	
							nmea_variation_receive_time=millisecond_tick_count;
						}
						nmea_variation_receive_time=millisecond_tick_count;
					}
					break;	
					
				case NMEA_GLL:
					util_get_field(6, (char *)(nmea_messages_in[message_position]+1), next_message_field, ',');
					if(*next_message_field!='A')
					{
						break;
					}	
					
					if(util_get_field(1, (char *)(nmea_messages_in[message_position]+1), next_message_field, ',')>0)
					{
						sscanf(next_message_field, "%f", &coord);
						nmea_latitude_degrees=(signed int)(coord/100.0f);
						nmea_latitude_minutes=100.0f*frac(coord/100.0f);
						
						util_get_field(2, (char *)(nmea_messages_in[message_position]+1), next_message_field, ',');
						if(*next_message_field=='N' || *next_message_field=='S')
						{
							if(*next_message_field=='S')
							{
								nmea_latitude_degrees=-nmea_latitude_degrees;
								nmea_latitude_minutes=-nmea_latitude_minutes;
							}	
							nmea_latitude_receive_time=millisecond_tick_count;
						}	
					}
					
					if(util_get_field(3, (char *)(nmea_messages_in[message_position]+1), next_message_field, ',')>0)
					{
						sscanf(next_message_field, "%f", &coord);
						nmea_longitude_degrees=(signed int)(coord/100.0f);
						nmea_longitude_minutes=100.0f*frac(coord/100.0f);
						
						util_get_field(4, (char *)(nmea_messages_in[message_position]+1), next_message_field, ',');
						if(*next_message_field=='E' || *next_message_field=='W')
						{
							if(*next_message_field=='W')
							{
								nmea_longitude_degrees=-nmea_longitude_degrees;
								nmea_longitude_minutes=-nmea_longitude_minutes;
							}	
							nmea_longitude_receive_time=millisecond_tick_count;
						}	
					}					
					
					if(util_get_field(5, (char *)(nmea_messages_in[message_position]+1), next_message_field, ',')>=6)
					{
						nmea_time.hour=(next_message_field[0]-'0')*10+(next_message_field[1]-'0');
						nmea_time.minute=(next_message_field[2]-'0')*10+(next_message_field[3]-'0');
						nmea_time.second=(next_message_field[4]-'0')*10+(next_message_field[5]-'0');
						if(nmea_time.hour<24 && nmea_time.minute<60 && nmea_time.second<60)
						{
							nmea_time_receive_time=millisecond_tick_count;
						}	
					}	
					break;	
					
				// nmea add more message types here
	
				default:
					break;
			}
		}
		
		// indicate that this message slot is ready to be written to again
		nmea_messages_in[message_position][0]=MS_DONE;	
	}	

	message_position++;
	if(message_position==NUMBER_NMEA_MESSAGES)
	{
		message_position=0;
	}
}

unsigned char nmea_test_checksum(char *message)
{
	unsigned char i,j;
	unsigned char checksum;
	unsigned char nmea_data_length;
	unsigned char received_checksum;

	// calculate checksum on received message
	checksum=0;

	if(util_get_field(0, message, next_message_field, '*')<5)
	{
	    return FALSE;
	}
	nmea_data_length=strlen(next_message_field)-1;
	for(i=0; i<nmea_data_length; i++)
	{
		checksum^=next_message_field[i+1];
	}

	// get checksum from received message
	if(util_get_field(1, message, next_message_field, '*')==0)
	{
		// no checksum so always ok
	    return TRUE;
	}

	if(strlen(next_message_field)!=2)
	{
	    return FALSE;
	}
	
	received_checksum=0;
	for(j=0; j<2; j++)
	{
		if(isdigit(next_message_field[j]))
		{
			received_checksum+=(next_message_field[j]-'0')<<((1-j)<<2);
		}
		else if(isalpha(toupper(next_message_field[j])))
		{
			received_checksum+=(toupper(next_message_field[j])-'A'+10)<<((1-j)<<2);
		}
		else
		{
			return FALSE;
		}
	}

	return received_checksum==checksum;
}

static void nmea_add_header(nmea_message_type_t message_type)
{
	strcpypgm2ram(nmea_message, "$II");
	strcat(nmea_message, nmea_get_name_from_type(message_type));
	strcatpgm2ram(nmea_message, ",");
}	

static void nmea_add_basic_float(float value)
{
	if(value!=FLT_MAX)
	{
		strcat(nmea_message, ftoa(value, 1));
	}
}	

static void nmea_add_commas(unsigned char c)
{
	while(c>0)
	{
		strcatpgm2ram(nmea_message, ",");
		c--;
	}	
}	

static void nmea_add_time(time_t time)
{
	if(time.hour!=UCHAR_MAX)
	{
		strcat(nmea_message, util_padded_uitoa(time.hour, 2));
		strcat(nmea_message, util_padded_uitoa(time.minute, 2));
		strcat(nmea_message, util_padded_uitoa(time.second, 2));
	}	
}

static void nmea_add_latitude(nav_data_t *nav_data)
{
	float latitude=fabs((float)nav_data->latitude_degrees*100.0f)+nav_data->latitude_minutes;
	
	if(nav_data->latitude_degrees!=INT_MAX)
	{
		strcat(nmea_message, util_padded_uitoa((unsigned int)latitude, 4));
		strcatpgm2ram(nmea_message, ".");
		strcat(nmea_message, util_padded_uitoa((unsigned int)((latitude*100.0f)-((unsigned long)latitude)*100), 2));
		if(nav_data->latitude_degrees<0)
		{
			strcatpgm2ram(nmea_message, ",S,");
		}		
		else
		{
			strcatpgm2ram(nmea_message, ",N,");
		}	
	}	
	else
	{
		nmea_add_commas(2);
	}	
}	

static void nmea_add_longitude(nav_data_t *nav_data)
{
	float longitude=fabs((float)nav_data->longitude_degrees*100.0f)+nav_data->longitude_minutes;

	if(nav_data->longitude_degrees!=INT_MAX)
	{
		strcat(nmea_message, util_padded_uitoa((unsigned int)longitude, 5));
		strcatpgm2ram(nmea_message, ".");
		strcat(nmea_message, util_padded_uitoa((unsigned int)((longitude*100.0f)-((unsigned long)longitude)*100), 2));
		if(nav_data->longitude_degrees<0)
		{
			strcatpgm2ram(nmea_message, ",W,");
		}		
		else
		{
			strcatpgm2ram(nmea_message, ",E,");
		}			
	}	
	else
	{
		nmea_add_commas(2);
	}	
}
	
void nmea_DPT_send(float depth)
{
	nmea_add_header(NMEA_DPT);
	nmea_add_basic_float(depth);
	nmea_add_commas(2);
	nmea_add_checksum_and_send(nmea_message);
}

void nmea_DBT_send(float depth)
{
	nmea_add_header(NMEA_DBT);
	nmea_add_basic_float(depth);
	strcatpgm2ram(nmea_message, ",M,,");
	nmea_add_checksum_and_send(nmea_message);
}

void nmea_VHW_send(float heading_true, float heading_magnetic, float boatspeed)
{
	nmea_add_header(NMEA_VHW);
	nmea_add_basic_float(heading_true);
	strcatpgm2ram(nmea_message, ",T,");
	nmea_add_basic_float(heading_magnetic);
	strcatpgm2ram(nmea_message, ",M,");
	nmea_add_basic_float(boatspeed);
	strcatpgm2ram(nmea_message, ",N,,K");
	nmea_add_checksum_and_send(nmea_message);
}

void nmea_RSA_send(float rudder)
{
	nmea_add_header(NMEA_RSA);
	nmea_add_basic_float(rudder);
	strcatpgm2ram(nmea_message, ",A,,");			
	nmea_add_checksum_and_send(nmea_message);
}

void nmea_HDM_send(float heading_magnetic)
{
	nmea_add_header(NMEA_HDM);
	nmea_add_basic_float(heading_magnetic);
	strcatpgm2ram(nmea_message, ",M");		
	nmea_add_checksum_and_send(nmea_message);
}

void nmea_HDG_send(float heading_magnetic, float variation)
{
	nmea_add_header(NMEA_HDG);
	nmea_add_basic_float(heading_magnetic);
	nmea_add_commas(3);
	if(variation!=FLT_MAX)
	{
		if(variation<0.0f)
		{
			strcat(nmea_message, ftoa(-variation, 0));
			strcatpgm2ram(nmea_message, ",W");
		}
		else
		{
			strcat(nmea_message, ftoa(variation, 0));
			strcatpgm2ram(nmea_message, ",E");
		}
	}
	else
	{
		nmea_add_commas(1);
	}

	nmea_add_checksum_and_send(nmea_message);
}

void nmea_HDT_send(float heading_true)
{
	nmea_add_header(NMEA_HDT);
	nmea_add_basic_float(heading_true);
	strcatpgm2ram(nmea_message, ",M");			
	nmea_add_checksum_and_send(nmea_message);
}

void nmea_MTW_send(float temperature)
{
	nmea_add_header(NMEA_MTW);
	nmea_add_basic_float(temperature);
	strcatpgm2ram(nmea_message, ",C");	
	nmea_add_checksum_and_send(nmea_message);
}	

void nmea_VLW_send(float trip, float log)
{
	nmea_add_header(NMEA_VLW);
	nmea_add_basic_float(log);
	strcatpgm2ram(nmea_message, ",N,");
	if(trip!=FLT_MAX)
	{
		strcat(nmea_message, ftoa(trip, 2));
	}
	strcatpgm2ram(nmea_message, ",N,,N,,N");

	nmea_add_checksum_and_send(nmea_message);
}	

void nmea_VWR_send(float apparent_wind_angle, float apparent_wind_speed)
{
	nmea_add_header(NMEA_VWR);
	if(apparent_wind_angle!=FLT_MAX)
	{
		if(apparent_wind_angle<180.0f)
		{
			strcat(nmea_message, ftoa(apparent_wind_angle, 0));
			strcatpgm2ram(nmea_message, ",R,");
		}
		else
		{
			strcat(nmea_message, ftoa(360.0f-apparent_wind_angle, 0));
			strcatpgm2ram(nmea_message, ",L,");
		}			
	}
	else
	{
		nmea_add_commas(2);
	}	
	
	nmea_add_basic_float(apparent_wind_speed);
	strcatpgm2ram(nmea_message, ",N,,M,,K");

	nmea_add_checksum_and_send(nmea_message);
}	

void nmea_VWT_send(float apparent_wind_angle, float apparent_wind_speed, float boatspeed)
{
	float true_wind_angle;
	float true_wind_speed;
	
	util_calc_true_wind_from_apparent(apparent_wind_speed, apparent_wind_angle, boatspeed, &true_wind_speed, &true_wind_angle);

	nmea_add_header(NMEA_VWT);	
	if(true_wind_angle<180.0f)
	{
		strcat(nmea_message, ftoa(true_wind_angle, 0));
		strcatpgm2ram(nmea_message, ",R,");
	}
	else
	{
		strcat(nmea_message, ftoa(360.0f-true_wind_angle, 0));
		strcatpgm2ram(nmea_message, ",L,");
	}			

	strcat(nmea_message, ftoa(true_wind_speed, 1));
	strcatpgm2ram(nmea_message, ",N,,M,,K");

	nmea_add_checksum_and_send(nmea_message);
}	

void nmea_MWV_send(float apparent_wind_angle, float apparent_wind_speed, float boatspeed)
{
	float true_wind_angle;
	float true_wind_speed;

	nmea_add_header(NMEA_MWV);	
	nmea_add_basic_float(apparent_wind_angle);	
	strcatpgm2ram(nmea_message, ",R,");
	nmea_add_basic_float(apparent_wind_speed);	
	strcatpgm2ram(nmea_message, ",N,A");

	nmea_add_checksum_and_send(nmea_message);
	
	if(apparent_wind_angle!=FLT_MAX && apparent_wind_speed!=FLT_MAX && boatspeed!=FLT_MAX)
	{
		util_calc_true_wind_from_apparent(apparent_wind_speed, apparent_wind_angle, boatspeed, &true_wind_speed, &true_wind_angle);
		nmea_add_header(NMEA_MWV);	
		strcat(nmea_message, ftoa(true_wind_angle, 0));
		strcatpgm2ram(nmea_message, ",T,");
		strcat(nmea_message, ftoa(true_wind_speed, 1));
		strcatpgm2ram(nmea_message, ",N,A");
	
		nmea_add_checksum_and_send(nmea_message);
	}
}	

void nmea_RMC_send(nav_data_t *nav_data)
{
	nmea_add_header(NMEA_RMC);	
	nmea_add_time(nav_data->time);
	strcatpgm2ram(nmea_message, ",A,");
	
	nmea_add_latitude(nav_data);
	nmea_add_longitude(nav_data);	
	nmea_add_basic_float(nav_data->sog);	
	nmea_add_commas(1);
	nmea_add_basic_float(nav_data->cog);
	nmea_add_commas(1);
	if(nav_data->date.year!=UCHAR_MAX)
	{
		strcat(nmea_message, util_padded_uitoa(nav_data->date.date, 2));
		strcat(nmea_message, util_padded_uitoa(nav_data->date.month, 2));
		strcat(nmea_message, util_padded_uitoa(nav_data->date.year, 2));
	}
	nmea_add_commas(1);
	if(nav_data->variation!=FLT_MAX)
	{
		strcat(nmea_message, ftoa(fabs(nav_data->variation),0));
		if(nav_data->variation<0.0f)
		{
			strcatpgm2ram(nmea_message, ",W,A");
		}	
		else
		{
			strcatpgm2ram(nmea_message, ",E,A");
		}	
	}	
	else
	{
		strcatpgm2ram(nmea_message, ",,A");	
	}	
	
	nmea_add_checksum_and_send(nmea_message);
}	

void nmea_GLL_send(nav_data_t *nav_data)
{
	nmea_add_header(NMEA_GLL);	
	nmea_add_latitude(nav_data);	
	nmea_add_longitude(nav_data);
	nmea_add_time(nav_data->time);
	strcatpgm2ram(nmea_message, ",A,A");
	
	nmea_add_checksum_and_send(nmea_message);
}	

// nmea add more message types here
