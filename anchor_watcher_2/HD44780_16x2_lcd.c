#include <p18cxxx.h>
#include <delays.h>
#include "hd44780_16x2_lcd.h"

#define E 				LATCbits.LATC0	
#define RS 				LATBbits.LATB2
#define RW				LATBbits.LATB3
#define BUSY			PORTAbits.RA7
#define BUSY_TRIS		TRISAbits.RA7
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
static void lcd_write_nibble(unsigned char c);
static void lcd_strobe(void);

static void lcd_strobe(void)
{
	E=ENABLED;
	Delay10TCYx(16);  
	E=DISABLED;
	Delay10TCYx(16);  
}
	
// write half a byte on data lines DB4, DB5, DB6, DB7
static void lcd_write_nibble(unsigned char c)
{
	LATAbits.LATA4=!!(c&0x01);
	LATAbits.LATA5=!!(c&0x02);
	LATAbits.LATA6=!!(c&0x04);
	LATAbits.LATA7=!!(c&0x08);
}

void lcd_set_cg_character(unsigned char position, const rom unsigned char *bytes)
{
	unsigned char row;
	unsigned char cg_address;
	
	if(position>15)
	{
		return;
	}	
	
	cg_address=position*8+0b01000000;
	lcd_write_command(cg_address);
	
	for(row=0; row<8; row++)
	{
		lcd_write_data(bytes[row]);
	}
}
	
void lcd_show_cg_character(unsigned char row, unsigned char column, unsigned char c)
{
	if(column>15 || row>1)
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
	
	lcd_write_data(c);
}	

void lcd_clear()
{
	lcd_clear_line(0);
	lcd_clear_line(1);
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
	
	lcd_write_nibble(c>>4);;
    lcd_strobe(); 
	lcd_write_nibble(c);
    lcd_strobe();
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
    
    // enable strobe 
    E=ENABLED;   
    
    // wait on busy flag
    while(BUSY);   
    
    // disable strobe
    E=DISABLED;   
    
    // set tristate back to output
    BUSY_TRIS=0;    

	Delay10TCYx(16); 
} 

// write a command byte to the display, strobing E each time to latch them in
static void lcd_write_command(unsigned char c)
{
	lcd_wait_until_not_busy();	
	
	// set r/w to write	
	RW=WRITE;			// RW=low
	
	// set data/command to command
	RS=COMMAND;			// RS=low
	
	lcd_write_nibble(c>>4);;
    lcd_strobe();
	lcd_write_nibble(c);
    lcd_strobe();
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
	TRISA&=0x0f;
	TRISCbits.RC0=0;
	TRISBbits.RB2=0;
	TRISBbits.RB3=0;
				
	E=DISABLED;							// E low 
	RS=COMMAND;   						// RS low
	RW=WRITE;							// RW low
   
    __delay_ms(15);	
    lcd_write_nibble(0x03);
    lcd_strobe();
    __delay_ms(5);		
    lcd_strobe();
	__delay_us(200);
    lcd_strobe();
 	__delay_us(200);
	lcd_write_nibble(0x02);	
    lcd_strobe();
	lcd_write_command(0x28);  
    __delay_us(40);      
    lcd_write_command(0x06);
    lcd_write_command(0x0c);   
    lcd_clear();   
}

void __delay_us(unsigned int c)
{
	unsigned int i;

	for(i=0; i<c; i++)
	{
		Delay1TCY();
		Delay1TCY();
		Delay1TCY();
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
		Delay1KTCYx(8);
	}
} 	
