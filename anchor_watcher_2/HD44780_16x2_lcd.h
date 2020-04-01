#ifndef _HD44780_16X2_LCD_H
#define _HD44780_16X2_LCD_H

void lcd_init(void);
void lcd_puts(unsigned char row, unsigned char column, const rom char *s);
void lcd_puts_ram(unsigned char row, unsigned char column, char *s);
void lcd_clear(void);
void lcd_clear_line(unsigned char row);
void __delay_ms(unsigned int c);
void __delay_us(unsigned int c);
void lcd_set_cg_character(unsigned char position, const rom unsigned char *bytes);
void lcd_show_cg_character(unsigned char row, unsigned char column, unsigned char c);

#endif