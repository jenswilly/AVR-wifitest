/*
 *  main.c
 *  wifitest
 *
 *  Created by Jens Willy Johannsen on 17-04-12.
 *  Copyright Greener Pastures 2012. All rights reserved.
 *
 */

/* Precompiler stuff for calculating USART baud rate value 
 * Otherwise, use this page: http://www.wormfood.net/avrbaudcalc.php?postbitrate=9600&postclock=12&bit_rate_table=on
 */
#define USART_BAUDRATE 9600	// error 0.2%
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1) 

// Set DDRx, PORTx, PINx and Pxy for debugging LED
#define DEBUG_DDR	DDRB
#define DEBUG_PORT	PORTB
#define DEBUG_BIT	PB5

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdarg.h>			// for va_ macros
#include <avr/pgmspace.h>	// for PSTR
#include "main.h"

// Global vars
#define RX_BUF_SIZE	1024
char usartBuffer[RX_BUF_SIZE];				// USART receive buffer
volatile unsigned char usartPtr;			// USART buffer pointer
volatile unsigned char responseCode;		// Last response code

// Serial stream
static int uart_putchar( char c, FILE *stream );
FILE usart = FDEV_SETUP_STREAM( uart_putchar, NULL, _FDEV_SETUP_WRITE );


// Interrupt handler for USART receive complete
ISR( USART_RX_vect ) 
{ 
	unsigned char byte;
	
	// Prevent buffer overflow
	if( usartPtr >= RX_BUF_SIZE-1 )
		usartPtr = 0;
	
	// Grab the data and but it into the buffer and increase pointer
	byte = UDR0;
	usartBuffer[ usartPtr++ ] = byte;

	/*
	// Ignore CR and LF
	if( byte != 0x0A && byte != 0x0D )
		usartBuffer[ usartPtr++ ] = byte;
	*/
	
	// Check for known status codes. Some commands might return a lot of text so we look at the last X chars and match against the known
	// response codes.
	if( usartPtr >= 6 && strncmp_P( usartBuffer + usartPtr-6, PSTR("\r\nOK\r\n"), 6 ) == 0 )
	{
		// Last 6 chars are \r\nOK\r\n: OK was received
		responseCode = S2W_SUCCESS;
		
		/// TODO: copy response buffer and send to parsing callback method if one is set
		/// …
		
		// Reset buffer
		usartPtr = 0; 
	}
	else if( usartPtr >= 9 && strncmp_P( usartBuffer + usartPtr-9, PSTR("\r\nERROR\r\n"), 9 ) == 0 )
	{
		responseCode = S2W_FAILURE;
		
		// Reset buffer
		usartPtr = 0;
	}
}

// Stream handler so we can use fprintf( &usart, ... ) to print serial data
static int uart_putchar(char c, FILE *stream)
{
	// Wait until we're ready to send
	while( !(UCSR0A & (1<< UDRE0)))
		;
	
	// Send byte
	UDR0 = c;
	return 0;
}

int send_serial_command_P( const prog_char *fmt, ... )
{
	responseCode = S2W_PENDING;
	va_list vl;
	va_start( vl, fmt );
	fprintf_P( &usart, fmt, vl );
	va_end( vl );
	
	// wait until receiveComplete
	while( responseCode == S2W_PENDING )
		;
	
	// Return response code
	return responseCode;
}

/* This method flashes the debugging LED the specified number of times.
 * Define DEBUG_PORT and DEBUG_BIT and set the pin as output beforehand.
 */
void debug_error( int errorNumber )
{
	int i;
	for( i=0; i<errorNumber; i++ )
	{
		// Flash debugging LED
		DEBUG_PORT |= (1<< DEBUG_BIT);
		_delay_ms( 100 );
		DEBUG_PORT &= ~(1<< DEBUG_BIT);
		_delay_ms( 100 );
	}
}

void setup_ports()
{
	// Debug LED -> output
	DEBUG_DDR |= (1<< DEBUG_BIT);
	DEBUG_PORT &= ~(1<< DEBUG_BIT);	// LED off
}

void setup_interrupts()
{
	// Enable USART RX interrupt
	UCSR0B |= (1<< RXCIE0);	// Enable USART RX interrupt
}

// Enables USART comm
// Remember to call this method after waking up
void setup_serial()
{
	UCSR0B |= (1<< RXEN0) | (1<< TXEN0);			// Enable TX and RX
	//	UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01);	// 8N1 serial format. Set by default
	UBRR0H = (BAUD_PRESCALE >> 8) & 0x0F;			// High part of baud rate masked so bits 15:12 are writted as 0 as per the datasheet (19.10.5)
	UBRR0L = BAUD_PRESCALE;							// Low part of baud rate. Writing to UBBRnL updates baud prescaler
}


int main(void)
{
	// Initialize
	setup_ports();
	setup_serial();
	setup_interrupts();
	sei();
	
	// Wait a bit to let the WiFi module power up
	_delay_ms( 1000 );
	
	// Let's see if it works at all
	if( send_serial_command_P( PSTR("AT\r") ) == S2W_SUCCESS )
		debug_error( 1 );
	else
		debug_error( 2 );
	
	// Main loop
	while(1)
	{
		
	}

    return 0;
}
