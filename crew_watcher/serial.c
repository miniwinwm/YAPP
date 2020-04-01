/* All references to section..., table... and figure... refer to the Microchip's processor datasheet */

#include <p18cxxx.h>
#include <delays.h>
#include "serial.h"

// TX1 is on port RC6 
// RX1 is on port RC7
void serial_setup(void)
{
	ANSELCbits.ANSC7=0;			// disable analog input on this pin, section 16.1.2.1
	BAUDCON1bits.BRG16=0;		// 8 bit baud rate generator, section 16.1.1.7
	SPBRG1=103;					// baud Rate register to 4800 baud at 64MHz oscillator, section 16.3
	TXSTA1bits.BRGH=0;			// set low generator calculation, section 16.3	
	TRISCbits.TRISC7=1;			// configure rx pin, section 16.1.1.7
	TXSTA1bits.SYNC=0;			// asynchronous mode, section 16.1.1.7
	RCSTA1bits.SPEN=1;			// enable the serial port, section 16.1.1.7
	TXSTA1bits.TX9=0;			// 8 bit data transmission, section 16.1.1.7
	RCSTA1bits.RX9=0;			// 8 bit data reception, section 16.1.2.7
	BAUDCON1bits.DTRXP=0;		// set receive polarity to normal, section 16.1.2.3
	TXSTA1bits.TXEN=0;			// enable transmission, section 16.1.1.7
	RCSTA1bits.CREN=1;			// enable reception, section 16.1.2.1
	PIE1bits.TX1IE=0;			// no serial port transmission interrupts, section 16.1.1.4
	PIE1bits.RC1IE=1;			// no serial port receive interrupts, section 16.1.2.4
	Delay100TCYx(160);			// 1ms delay to allow settings to take place
}

