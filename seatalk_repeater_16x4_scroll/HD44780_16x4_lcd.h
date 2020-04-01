#ifndef _HD44780_16X4_LCD_H
#define _HD44780_16X4_LCD_H

void lcd_init(void);
void lcd_puts(unsigned char row, const rom char *s);
void lcd_puts_ram(unsigned char row, char *s);
void lcd_clear(void);
void lcd_clear_line(unsigned char row);
void __delay_ms(unsigned int c);
void __delay_us(unsigned int c);

#endif