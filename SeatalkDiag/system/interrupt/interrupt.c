#include <p18cxxx.h>
#include "user/seatalk.h"

extern volatile unsigned char messages[NUMBER_OF_MESSAGES][MAX_MESSAGE_SIZE+1];
extern volatile unsigned long second_tick_count;

void low_isr(void);
#pragma code low_vector=0x08
void low_interrupt(void)
{
	_asm GOTO low_isr _endasm
}
	
#pragma code
#pragma interruptlow low_isr
void low_isr(void)
{
	static unsigned int tick_count=0;

	tick_count++;
	if(tick_count==38400)
	{
		tick_count=0;
		second_tick_count++;
	}

	do_seatalk_read();	
}