#include <string.h>
#include <ctype.h>
#include "nmea.h"
#include "mathematics.h"
#include "sscanf.h"
#include "gps_repeater.h"

extern volatile char nmea_messages[NUMBER_NMEA_MESSAGES][NMEA_MESSAGE_MAX_LENGTH+1];
extern char next_message_field[83];
extern float latitude_dm;
extern float longitude_dm;
extern float latitude_dd;
extern float longitude_dd;
extern float sog;
extern float cog;
extern volatile unsigned long one_second_tick_count;
extern unsigned long position_timestamp;
extern unsigned long cog_sog_timestamp;

unsigned char process_vtg_message(unsigned char message_number)
{
	get_field(9, (char *)(nmea_messages[message_number]+1), next_message_field, ',');
	if(strchrpgm("ADP", next_message_field[0]))
	{	
		// sog
		get_field(5, (char *)(nmea_messages[message_number]+1), next_message_field, ',');
		sscanf(next_message_field, "%f", &sog);
		
		// cog
		get_field(1, (char *)(nmea_messages[message_number]+1), next_message_field, ',');
		sscanf(next_message_field, "%f", &cog);				
				
		// update time of fix
		cog_sog_timestamp=one_second_tick_count;
	}
	else
	{
		cog_sog_timestamp=0L;
	}		

	return TRUE;
}	

unsigned char process_rmc_message(unsigned char message_number)
{
	get_field(2, (char *)(nmea_messages[message_number]+1), next_message_field, ',');
	if(next_message_field[0]=='A')
	{
		// latitude
		get_field(3, (char *)(nmea_messages[message_number]+1), next_message_field, ',');
		sscanf(next_message_field, "%f", &latitude_dm);
		latitude_dm/=100.0;
		get_field(4, (char *)(nmea_messages[message_number]+1), next_message_field, ',');
		if(next_message_field[0]=='S')
		{
			latitude_dm=-latitude_dm;
		}	
		latitude_dd=convert_degrees_frac_minutes_2_degrees_frac_degrees(latitude_dm);
		
		// longitude
		get_field(5, (char *)(nmea_messages[message_number]+1), next_message_field, ',');
		sscanf(next_message_field, "%f", &longitude_dm);
		longitude_dm/=100.0;
		get_field(6, (char *)(nmea_messages[message_number]+1), next_message_field, ',');
		if(next_message_field[0]=='W')
		{
			longitude_dm=-longitude_dm;
		}	
		longitude_dd=convert_degrees_frac_minutes_2_degrees_frac_degrees(longitude_dm);
		
		// sog
		get_field(7, (char *)(nmea_messages[message_number]+1), next_message_field, ',');
		sscanf(next_message_field, "%f", &sog);
		
		// cog
		get_field(8, (char *)(nmea_messages[message_number]+1), next_message_field, ',');
		sscanf(next_message_field, "%f", &cog);				
				
		// update time of fix
		position_timestamp=one_second_tick_count;	
		cog_sog_timestamp=one_second_tick_count;
	}
	else
	{
		position_timestamp=0L;
		cog_sog_timestamp=0L;
	}		

	return TRUE;
}	

unsigned char process_gll_message(unsigned char message_number)
{
	get_field(6, (char *)(nmea_messages[message_number]+1), next_message_field, ',');
	if((strlen(next_message_field)>0 && next_message_field[0]=='A') || (strlen(next_message_field)==0))
	{
		// latitude
		get_field(1, (char *)(nmea_messages[message_number]+1), next_message_field, ',');
		sscanf(next_message_field, "%f", &latitude_dm);
		latitude_dm/=100.0;
		get_field(2, (char *)(nmea_messages[message_number]+1), next_message_field, ',');
		if(next_message_field[0]=='S')
		{
			latitude_dm=-latitude_dm;
		}	
		latitude_dd=convert_degrees_frac_minutes_2_degrees_frac_degrees(latitude_dm);
		
		// longitude
		get_field(3, (char *)(nmea_messages[message_number]+1), next_message_field, ',');
		sscanf(next_message_field, "%f", &longitude_dm);
		longitude_dm/=100.0;
		get_field(4, (char *)(nmea_messages[message_number]+1), next_message_field, ',');
		if(next_message_field[0]=='W')
		{
			longitude_dm=-longitude_dm;
		}	
		longitude_dd=convert_degrees_frac_minutes_2_degrees_frac_degrees(longitude_dm);			
				
		// update time of fix
		position_timestamp=one_second_tick_count;						
	}
	else
	{
		position_timestamp=0L;
	}		

	return TRUE;
}	

unsigned char process_gga_message(unsigned char message_number)
{
	get_field(6, (char *)(nmea_messages[message_number]+1), next_message_field, ',');
	if(next_message_field[0]>='1' && next_message_field[0]<='5')
	{
		// latitude
		get_field(2, (char *)(nmea_messages[message_number]+1), next_message_field, ',');
		sscanf(next_message_field, "%f", &latitude_dm);
		latitude_dm/=100.0;
		get_field(3, (char *)(nmea_messages[message_number]+1), next_message_field, ',');
		if(next_message_field[0]=='S')
		{
			latitude_dm=-latitude_dm;
		}	
		latitude_dd=convert_degrees_frac_minutes_2_degrees_frac_degrees(latitude_dm);
		
		// longitude
		get_field(4, (char *)(nmea_messages[message_number]+1), next_message_field, ',');
		sscanf(next_message_field, "%f", &longitude_dm);
		longitude_dm/=100.0;
		get_field(5, (char *)(nmea_messages[message_number]+1), next_message_field, ',');
		if(next_message_field[0]=='W')
		{
			longitude_dm=-longitude_dm;
		}	
		longitude_dd=convert_degrees_frac_minutes_2_degrees_frac_degrees(longitude_dm);			
				
		// update time of fix
		position_timestamp=one_second_tick_count;						
	}
	else
	{
		position_timestamp=0L;
	}		

	return TRUE;
}	

/** Test that a NMEA message has a valid checksum */
unsigned char test_nmea_checksum(char *message)
{
	unsigned char i,j;
	unsigned char checksum;
	unsigned char nmea_data_length;
	unsigned char received_checksum;

	// calculate checksum on received message
	checksum=0;
	get_field(0, message, next_message_field, '*');
	if(strlen(next_message_field)<5)
	{
	    return FALSE;
	}
	nmea_data_length=strlen(next_message_field)-1;
	for(i=0; i<nmea_data_length; i++)
	{
		checksum^=next_message_field[i+1];
	}

	// get checksum from received message
	get_field(1, message, next_message_field, '*');

	if(strlen(next_message_field)==0)
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

/** Get the n'th field from a message using a given field separator */
void get_field(unsigned char field_number, char *message, char *message_field, char delimeter)
{
	unsigned char delimeter_count=0;
	unsigned char next_char=0;
	unsigned char message_length;
	unsigned char field_length=0;

	message_length=strlen(message);

	// find start of field
	while(1)
	{
	    if(delimeter_count==field_number)
	    {
	        break;
	    }

		next_char++;
		if(next_char==message_length)
		{
		    message_field[0]='\0';
		    return;
		}

		if(message[next_char]==delimeter)
		{
		    delimeter_count++;
		}
	}

	// move over delimeter
	if(next_char>0)
	{
		next_char++;
	}

	// find field length
	while(1)
	{
	    if(message[next_char+field_length]==delimeter || message[next_char+field_length]=='\0')
	    {
	        break;
	    }
	    field_length++;
	}

	strncpy(message_field, message+next_char, field_length);
	message_field[field_length]=0;
}