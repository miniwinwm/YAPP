#include <p18cxxx.h>
#include <delays.h>
#include "nrf24l01.h"
#include "eeprom.h"

// Configuration bits are described in section 24.1
// Note: For a complete list of the available config pragmas and their values, in MPLAB
// click Help|Topics then select "PIC18 Config Settings" in the Language Tools section.
#pragma config FOSC=INTIO67		// Internal oscillator block, port function on RA6 and RA7 
#pragma config PLLCFG=OFF		// Oscillator PLL set in software for HFINTOSC  
#pragma config PRICLKEN=OFF		// Primary clock disabled, using INTOSC  
#pragma config FCMEN=OFF		// Fail-Safe Clock Monitor disabled  
#pragma config IESO=OFF			// Oscillator Switchover mode disabled  
#pragma config PWRTEN=OFF		// Power up timer disabled  
#pragma config BOREN=OFF		// Brown-out Reset enabled in hardware only (SBOREN is disabled) 
#pragma config BORV=285			// VBOR set to 2.85 V nominal  
#pragma config WDTEN=ON			// WDT enabled
#pragma config WDTPS=1024		// Watchdog timer prescalar 4ms units  
#pragma config CCP2MX=PORTC1	// CCP2 input/output is multiplexed with RC1  
#pragma config PBADEN=OFF		// PORTB<5:0> pins are configured as digital I/O on Reset  
#pragma config CCP3MX=PORTC6	// P3A/CCP3 input/output is mulitplexed with RC6  
#pragma config HFOFST=OFF		// HFINTOSC output and ready status are delayed by the oscillator stable status  
#pragma config T3CMX=PORTC0		// T3CKI is on RC0  
#pragma config P2BMX=PORTC0		// P2B is on RC0  
#pragma config MCLRE=EXTMCLR	// MCLR pin enabled, RE3 input pin disabled  
#pragma config STVREN=OFF		// Stack full/underflow will not cause Reset
#pragma config LVP=OFF			// Single-Supply ICSP disabled  
#pragma config XINST=OFF		// Instruction set extension and Indexed Addressing mode disabled (Legacy mode)  
#pragma config CP0=OFF			// Block 0 (000800-003fffh) not code-protected  
#pragma config CP1=OFF			// Block 1 (004000-007fffh) not code-protected  
#pragma config CP2=OFF			// Block 2 (008000-00bfffh) not code-protected  
#pragma config CP3=OFF			// Block 3 (00c000-00ffffh) not code-protected  
#pragma config CPB=OFF			// Boot block (000000-0007ffh) not code-protected  
#pragma config CPD=OFF			// Data EEPROM not code-protected  
#pragma config WRT0=OFF			// Block 0 (000800-003fffh) not write-protected  
#pragma config WRT1=OFF			// Block 1 (004000-007fffh) not write-protected  
#pragma config WRT2=OFF			// Block 2 (008000-00bfffh) not write-protected  
#pragma config WRT3=OFF			// Block 3 (00c000-00ffffh) not write-protected  
#pragma config WRTC=OFF			// Configuration registers (300000-3000ffh) not write-protected  
#pragma config WRTB=OFF			// Boot Block (000000-0007ffh) not write-protected  
#pragma config WRTD=OFF			// Data EEPROM not write-protected  
#pragma config EBTR0=OFF		// Block 0 (000800-003fffh) not protected from table reads executed in other blocks  
#pragma config EBTR1=OFF		// Block 1 (004000-007fffh) not protected from table reads executed in other blocks  
#pragma config EBTR2=OFF		// Block 2 (008000-00bfffh) not protected from table reads executed in other blocks  
#pragma config EBTR3=OFF		// Block 3 (00c000-00ffffh) not protected from table reads executed in other blocks  
#pragma config EBTRB=OFF		// Boot Block (000000-0007ffh) not protected from table reads executed in other blocks  

unsigned char send_confirmation_to_base(unsigned char data);
unsigned char transmit_power;		// read from jumpers

// 13uA in sleep

#pragma romdata eedata_scn=0xf00000
rom unsigned char eedata_values[1] = {0};

#pragma code

//initialize routine
void initialize(void) 
{ 
	unsigned char power_register;
	
	// set all pins as digital i/o
	ANSELA=0;
	ANSELB=0;
	ANSELC=0;
	
	// set all unused pins as output and high
	TRISA=0;
	LATA=0xff;
	TRISCbits.TRISC0=0;
	LATCbits.LATC0=1;
	TRISCbits.TRISC6=0;
	LATCbits.LATC6=1;
	TRISCbits.TRISC7=0;
	LATCbits.LATC7=1;
	TRISBbits.TRISB1=0;
	LATBbits.LATB1=1;
	TRISBbits.TRISB2=0;
	LATBbits.LATB2=1;
	TRISBbits.TRISB3=0;
	LATBbits.LATB3=1;
	TRISBbits.TRISB6=0;
	LATBbits.LATB6=1;
	TRISBbits.TRISB7=0;
	LATBbits.LATB7=1;

	// get the transmit power level	
	TRISBbits.TRISB4=1;
	TRISBbits.TRISB5=1;
	WPUBbits.WPUB4=1;			// pull ups enabled on jumper pins
	WPUBbits.WPUB5=1;
	INTCON2bits.RBPU=0;			// enable all port b pull ups
	
	transmit_power=0;
	if(PORTBbits.RB5)
	{
		transmit_power+=1;
	}
	if(PORTBbits.RB4)
	{
		transmit_power+=2;
	}
	
	TRISBbits.TRISB4=0;
	LATBbits.LATB4=0;
	TRISBbits.TRISB5=0;
	LATBbits.LATB5=0;
	INTCON2bits.RBPU=1;			// disable all port b pull ups
	
	// set state of miso pin used during sleep
	LATCbits.LATC4=1;
	
	// IRQ=B0
	TRISBbits.TRISB0=1; 		// set irq pin as input
	
	// CSN=C2
	TRISCbits.TRISC2=0;			// set csn pin as output
	
	// CE=C1
	TRISCbits.TRISC1=0;			// set ce pin as output
	
	LATCbits.LATC2=1; 			// set CSN bit
	
	// init the spi port
	SSP1CON1bits.SSPEN=0;		// disable spi1
	
	// SDO1=C5
	TRISCbits.TRISC5=0;			// set mosi pin as output
	
	// SDI1=C4
	TRISCbits.TRISC4=1;      	// set miso pin as input
	
	// SCK1=C3
	TRISCbits.TRISC3=0;      	// set clk pin as output
	
	// setup spi parameters
	SSP1CON1bits.SSPM=0;
	SSP1CON1bits.CKP=0;	
	SSP1STATbits.SMP=0;	
	SSP1STATbits.CKE=1;	
	
	SSP1CON1bits.SSPEN=1; 		// enable spi1
	
	// init the rf chip as transmitter
	nrf24l01_initialize_debug(false, 1, false); 
	
	// set transmit power
	// nrf24l01_RF_SETUP_RF_PWR_0	highest power
	// nrf24l01_RF_SETUP_RF_PWR_6 	
	// nrf24l01_RF_SETUP_RF_PWR_12	
	// nrf24l01_RF_SETUP_RF_PWR_18	lowest power	
	
	nrf24l01_read_register(nrf24l01_RF_SETUP, &power_register, 1);
	power_register&=~nrf24l01_RF_SETUP;
	switch(transmit_power)
	{
		case 0:
			// both jumpers in place, lowest power
			power_register|=nrf24l01_RF_SETUP_RF_PWR_18;
			break;
			
		case 1:
			power_register|=nrf24l01_RF_SETUP_RF_PWR_12;		
			break;
			
		case 2:
			power_register|=nrf24l01_RF_SETUP_RF_PWR_6;		
			break;
			
		case 3:
			// no jumpers in place, highest power
			power_register|=nrf24l01_RF_SETUP_RF_PWR_0;			
			break;
	}	
	nrf24l01_write_register(nrf24l01_RF_SETUP, &power_register, 1);
} 

void main (void) 
{ 
	unsigned char address; 	// address to transmit, read from eeprom, 0-3

	OSCCONbits.IRCF=5;			// internal oscillator 4MHz, section 2.2.2
	OSCCONbits.IDLEN=0;			// SLEEP enters sleep mode, section 2.2.4
	OSCCONbits.SCS=0;			// system clock determined by config bits, section 2.3
	OSCTUNEbits.PLLEN=1;
	
	//initialize IO, SPI, set up nRF24L01 as transmitter
	initialize();  

	address=int_EEPROM_getc(0);
	
	while(1)
	{
		nrf24l01_power_down();		// put rf module into power down mode
		TRISCbits.TRISC4=0;			// set miso pin as output
		Sleep();
		TRISCbits.TRISC4=1; 		// set miso as input again
		nrf24l01_power_up(false);	// power up rf module, receiver off
		send_confirmation_to_base(address);
	}
}

unsigned char send_confirmation_to_base(unsigned char data)
{
	unsigned int tries;
	unsigned char success=0;
	
	nrf24l01_write_tx_payload(&data, 1, true); // transmit received char over rf 
	
	for(tries=0; tries<50; tries++)
	{
		// check to see if the data has been sent
		if(nrf24l01_irq_pin_active() && nrf24l01_irq_tx_ds_active())
		{
			// data sent, no more tries
			success=1;
			break;
		}
		
		Delay100TCYx(80); //delay 1ms
	}	
	
	nrf24l01_flush_tx();					// make sure everything is sent by rf module
	nrf24l01_irq_clear_all(); 				// clear rf module interrupts 

	return success;
}



