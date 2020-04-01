#include <p18cxxx.h>

unsigned char spi1_send_read_byte(unsigned char byte)
{
	SSPBUF=byte;
	
	while(!SSP1STATbits.BF);
	
	return SSPBUF;
}	
