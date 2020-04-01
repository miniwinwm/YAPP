#include <p18cxxx.h>
#include <delays.h>

/**
 * This function writes data into an address given in the internal EEPROM of the processor
 *
 * @param address - the address to write to
 * @param data - the value to write
 */
void int_EEPROM_putc(unsigned char address, unsigned char data)
{
    unsigned char intcon_save;

    EEADR=address;
    EEDATA=data;

    EECON1bits.EEPGD=0;			// 0 = Access data EEPROM memory
    EECON1bits.CFGS=0;			// 0 = Access Flash program or DATA EEPROM memory
    EECON1bits.WREN=1;			// enable writes to internal EEPROM

    intcon_save=INTCON;			// Save INTCON register contents
    INTCON=0;					// Disable interrupts, Next two lines SHOULD run without interrupts
    
    EECON2=0x55;				// Required sequence for write to internal EEPROM
    EECON2=0xaa;				// Required sequence for write to internal EEPROM

    EECON1bits.WR=1;			// begin write to internal EEPROM
    INTCON=intcon_save;			// Now we can safely enable interrupts if previously used
    
    Delay1TCY();

    while(PIR2bits.EEIF==0)		//Wait till write operation complete
    {
        Delay1TCY();
    }

    EECON1bits.WREN=0;			// Disable writes to EEPROM on write complete (EEIF flag on set PIR2 )
    PIR2bits.EEIF=0;			// Clear EEPROM write complete flag. (must be cleared in software. So we do it here)
}

/**
 * This function reads data from address given in internal EEPROM of the processor
 *
 * @param address - the address to read from
 * @return - the read value
 */
unsigned char int_EEPROM_getc(unsigned char address)
{
    EEADR=address;              // set EEPROM address to read from, section 7.2
    EECON1bits.EEPGD=0;			// access data EEPROM memory, section 7.2, register 7-1
    EECON1bits.CFGS=0;			// access flash program or data EEPROM memory, section ???????????, register 7-1
    EECON1bits.RD=1;			// read from EEPROM into EEDATA,
    return EEDATA;              // return data, section ??????????????, register 7-1
}

