#ifndef _SSCANF_H
#define _SSCANF_H

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

unsigned char sscanf(char *buf, const rom char *format, ...);
float my_atof(char *s, char **endptr);
unsigned long my_atoul(char *s, char **endptr, int base);
void ftoa(float value, char *s, int places);

#endif