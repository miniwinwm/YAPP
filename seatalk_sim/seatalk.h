#ifndef _SOFT_UART_H
#define _SOFT_UART_H

#define SEATALK_DATA_READ		PORTBbits.RB5
#define SEATALK_TRISB_READ_VAL 	0b00100000
#define SEATALK_ANSEL_B_VAL 	0b11011111
#define SEATALK_DATA_WRITE 		LATAbits.LATA3
#define SEATALK_TRISA_WRITE_VAL	0b11110111
#define SEATALK_ANSEL_A_VAL 	0b11110111
	
void init_seatalk(void);
unsigned char write_seatalk_byte(unsigned char byte, unsigned char command);
unsigned char write_seatalk_sentence(unsigned char length, unsigned char* command);

#ifndef TRUE
#define TRUE	1
#endif
#ifndef FALSE
#define FALSE 	0
#endif

#endif