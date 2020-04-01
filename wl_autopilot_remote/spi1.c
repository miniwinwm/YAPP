/*
	MAST DISPLAY

    Copyright (C) 2013  John Blaiklock

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <p18cxxx.h>

unsigned char spi1_send_read_byte(unsigned char byte)
{
	SSPBUF=byte;
	
	while(!SSP1STATbits.BF);
	
	return SSPBUF;
}	
