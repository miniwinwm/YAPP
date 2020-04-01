#include <p18cxxx.h>
#include <delays.h>
#include "hd44780_16x2_lcd.h"

#define E 				LATCbits.LATC0	
#define RS 				LATCbits.LATC4
#define RW				LATCbits.LATC5
#define BUSY			PORTAbits.RA7
#define BUSY_TRIS		TRISAbits.RA7
#define DATA_PORT		LATA
#define DATA			1
#define COMMAND			0
#define READ			1
#define WRITE			0
#define ENABLED 		1
#define DISABLED		0

static void lcd_wait_until_not_busy();
static void lcd_write_command(unsigned char c);
static void lcd_write_data(unsigned char c);
static void lcd_wait_until_not_busy();

void lcd_clear()
{
	lcd_write_command(0x01);
}	

void lcd_clear_line(unsigned char row)
{
	lcd_puts(row, 0, "                ");
}

// write a data byte to the display, strobing E to latch it in
static void lcd_write_data(unsigned char c)
{
	// wait until previous operation is finished	
	lcd_wait_until_not_busy();		
	// set r/w to write
	RW=WRITE;			// RW=low
	// set data/command to data	
	RS=DATA;			// RS=high
	DATA_PORT=c;
	E=ENABLED;
	E=DISABLED;	
}

// wait until the display indicates that the previous operation has finished by looking at busy bit on DB7
static void lcd_wait_until_not_busy()   
{   
	// set data/command to command		
    RS=COMMAND;   		// RS=low
	// set r/w to read    
    RW=READ;   			// RW=high
    // set tristate to input
    BUSY_TRIS=1;
    __delay_us(300);	    
    // enable strobe 
    E=ENABLED;   
    // wait on busy flag
    while(BUSY);   
    // disable strobe
    E=DISABLED;   
    // set tristate back to output
    BUSY_TRIS=0;    
} 

// write a command byte to the display, strobing E each time to latch them in
static void lcd_write_command(unsigned char c)
{
	lcd_wait_until_not_busy();	
	// set r/w to write	
	RW=WRITE;			// RW=low
	// set data/command to command
	RS=COMMAND;			// RS=low
	DATA_PORT=c;
	E=ENABLED;
	E=DISABLED;	
}

void lcd_puts_ram(unsigned char row, unsigned char column, char *s)
{
    unsigned char i; 

	if(row>1 || column>15)
	{
		return;
	}
  
    switch(row)   
    {   
        case 0:   
            lcd_write_command(0x80+column);   
            break;   
            
        case 1:   
            lcd_write_command(0xc0+column);   
            break;    
    }   
    
    for(i=0; s[i]; i++)   
    {   
        lcd_write_data(s[i]);   
    }   
}   

void lcd_puts(unsigned char row, unsigned char column, const rom char *s)
{   
    unsigned char i;   

	if(row>1 || column>15)
	{
		return;
	}

    switch(row)   
    {   
        case 0:   
            lcd_write_command(0x80+column);   
            break;   
            
        case 1:   
            lcd_write_command(0xc0+column);   
            break;   
    }   
    
    for(i=0; s[i]; i++)   
    {   
        lcd_write_data(s[i]);   
    }   
}   
	
// initialise the lcd and the PIC I/O to write mode - 8 bit data mode
void lcd_init()
{
	TRISA=0;		
	ANSELA=0;
	ANSELC&=0b11001111;
	TRISCbits.RC0=0;
	TRISCbits.RC4=0;
	TRISCbits.RC5=0;
				
	E=DISABLED;							// start off with E off 

    __delay_ms(15);   
    lcd_write_command( 0x38 );   
    __delay_ms(5);   
    lcd_write_command( 0x38 );    
    __delay_us(100);   
    lcd_write_command( 0x38 );  
    __delay_us(40);   
    lcd_write_command( 0x38 );  
    __delay_us(40);      
    lcd_write_command( 0x06 );
    __delay_us(40);          
    lcd_write_command( 0x0c );   
    __delay_us(130);   
    lcd_write_command( 0x01 );    
    __delay_ms(2);   
}

void __delay_us(unsigned int c)
{
	unsigned int i;

	for(i=0; i<c; i++)
	{
		Delay1TCY();
		Delay1TCY();
		Delay1TCY();
	}
} 	

void __delay_ms(unsigned int c)
{
	unsigned int i;
	
	for(i=0; i<c; i++) 
	{
		Delay1KTCYx(4);
	}
} 	
