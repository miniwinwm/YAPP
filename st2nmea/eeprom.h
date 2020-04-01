#ifndef _EEPROM_H
#define _EEPROM_H

unsigned char int_EEPROM_getc(unsigned char address);
void int_EEPROM_putc(unsigned char address, unsigned char data);

#endif