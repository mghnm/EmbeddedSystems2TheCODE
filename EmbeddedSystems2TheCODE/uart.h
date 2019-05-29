#ifndef _UART_H_
#define _UART_H_

#include <avr/io.h> 
/*#include <util/delay.h> 
#include <stdio.h> 
#include <stdlib.h> */



#ifndef F_CPU
/* prevent compiler error by supplying a default */
# warning "F_CPU not defined. Define F_CPU with 16 Mhz."
#define F_CPU 16000000UL    // Quarz with 16 Mhz  */
//#define F_CPU 12000000UL    // Quarz with 16 Mhz  */
#endif


#define UART_BAUDRATE0 9600
#define UART_BAUD_SELECT0 ((F_CPU / (UART_BAUDRATE0 * 16L))-1) 

/*#define UART_BAUDRATE1 9600
#define UART_BAUD_SELECT1 ((F_CPU / (UART_BAUDRATE0 * 16L))-1) */


/******************************************************************//**
 * Sends a single character via UART0 (eg. to a Hyperterminal).
 *********************************************************************/
void initUART0()
{	
	UBRR0H = (UART_BAUD_SELECT0>>8);
  	UBRR0L = UART_BAUD_SELECT0;
	UCSR0A = 0;
	// No double speed      see ATMEGA168A datasheet page 185
	UCSR0B = (1<<RXCIE0) | (1<<RXEN0) | (1<<TXEN0);	//Enable receiver, receiver interrupt, transmitter
	UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);				// frame format: 8data,  1 stop bit,  no parity
	UCSR0A |= (1<<TXC0);							// clear any existing transmits	
}

/******************************************************************//**
 * Sends a single character via UART0 (eg. to a Hyperterminal).
 *********************************************************************/
void uart0_putc(unsigned char c)   //USART_Receive0
{
    while (!(UCSR0A & (1<<UDRE0)))  // wait until sending is possible
    {;}
    UDR0 = c;                       // sending signs
}


/******************************************************************//**
 * Sends a complete string via UART (eg. to a Hyperterminal).
 *********************************************************************/
void uart0_puts (char *s)
{
    while (*s)
    {                               // sending char until "/0"
        uart0_putc(*s);
        s++;
    }
}


/******************************************************************//**
 * Sends an Integer via UART (eg. to a Hyperterminal).
 *********************************************************************/
void uart0_puti16 (uint16_t value)
{
	char _buffer[6];
	itoa( value, _buffer, 10 );	//conversion from integer to char
	uart0_puts(_buffer);
}

/******************************************************************//**
 * Sends an Integer via UART (eg. to a Hyperterminal).
 *********************************************************************/
/*void uart1_puti16 (uint16_t value)
{
	char _buffer[6];
	itoa( value, _buffer, 10 );	//conversion from integer to char
	uart1_puts(_buffer);
}*/

/******************************************************************//**
 * Sends an Integer in Binaryformat via UART (eg. to a Hyperterminal).
 *********************************************************************/
void uart0_puti16B (uint16_t value)
{
	char _buffer[17];
	itoa( value, _buffer, 2 );	//conversion from integer to char
	uart0_puts(_buffer);
}
/******************************************************************//**
 * Sends an Integer in Binaryformat via UART (eg. to a Hyperterminal).
 *********************************************************************/
/*void uart1_puti16B (uint16_t value)
{
	char _buffer[17];
	itoa( value, _buffer, 2 );	//conversion from integer to char
	uart1_puts(_buffer);
}*/

#endif /* _UART_H_ */
