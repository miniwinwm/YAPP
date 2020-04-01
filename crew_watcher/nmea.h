#ifndef _NMEA_H
#define _NMEA_H

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE 1
#endif

unsigned char test_nmea_checksum(char *ais_message);
void get_field(unsigned char field_number, char *message, char *message_field, char delimeter);
void process_rmc_message(unsigned char message_number);
void process_gll_message(unsigned char message_number);
void process_gga_message(unsigned char message_number);

#endif