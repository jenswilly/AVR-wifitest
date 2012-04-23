/*
 *  main.c
 *  wifitest
 *
 *  Created by Jens Willy Johannsen on 17-04-12.
 *  Copyright Greener Pastures 2012. All rights reserved.
 *
 */

/*
 * AT					// Are we live?
 * AT+WM=1				// Mode=ad hoc
 * AT+DHCPSRVR=1		// Enable DHCP server
 * AT+WAUTH=2			// WEP security
 * AT+WWEP1=616c69656e	// passphrase = 'alien'
 * AT+WA=GS1011M		// Create ad hoc network named 'GS1011M'
 * AT+NSUDP=49000		// Listen on UDP port 49000
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
volatile int usartPtr;						// USART buffer pointer
volatile int usartLinePtr;					// USART buffer pointer to beginning of line
volatile unsigned char responseCode;		// Last response code
void (*lineHandler)(const char*) = 0;		// Line handler function pointer (returns nothing; takes a pointer to immutable chars)
void (*udpHandler)(const char*) = 0;		// UDP server data handler function pointer (returns nothing; takes a pointer to immutable chars)

// Serial stream
static int uart_putchar( char c, FILE *stream );
FILE usart = FDEV_SETUP_STREAM( uart_putchar, NULL, _FDEV_SETUP_WRITE );


// Interrupt handler for USART receive complete
ISR( USART_RX_vect ) 
{ 
	unsigned char byte;
	int i;
	
	// Prevent buffer overflow
	if( usartPtr >= RX_BUF_SIZE-1 )
		resetBuffer();
	
	// Grab the data and but it into the buffer and increase pointer
	byte = UDR0;
	usartBuffer[ usartPtr++ ] = byte;

	// If we have received \n, we have en entire line
//	if( byte == '\n' )
//	{
//		// If we have a line handler, pass the line to be handled
//		if( lineHandler != 0 )
//			lineHandler( usartBuffer+usartLinePtr );	// Pass a pointer to the beginning of the current line
//		
//		// New line begins here
//		usartLinePtr = usartPtr;
//	}
	
	if( byte == 'E' && usartBuffer[ usartPtr-2 ] == 27 )
	{
		// <esc>E received: end of UDP server reception. Do we have a UDP server handler?
		if( udpHandler != 0 )
		{
			// Yes: scan back until we find the begin marker <esc>u
			for( i=usartPtr; i >= 0; i-- )
				if( usartBuffer[ i ] == 27 && usartBuffer[ i+1 ] == 'u' )
					break;
			
			// Make sure the string is 0-terminated
			usartBuffer[ usartPtr ] = 0;
			
			// Pass it to the handler
			udpHandler( usartBuffer+i );
			
			// Reset buffer
			usartPtr = 0;
		}
	}
	
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
	// 
	else if( usartPtr >= 24 && strncmp_P( usartBuffer + usartPtr-24, PSTR("\r\nERROR: INVALID INPUT\r\n"), 24 ) == 0 )
	{
		responseCode = S2W_EINVAL;
		
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

void resetBuffer()
{
	usartPtr = 0;							// Point af beginning of buffer
	usartLinePtr = 0;						// Ditto for line pointer
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

void receiveUdp( const char* buffer )
{
	int indx = 0;
	int indx2 = 0;
	char host[16];
	char msg[32];
	
	// We get the entire <esc>u<CID><source IP address><space><port><htab><data><esc>E string and we are only interested in source IP and data.

	// Scan past leading <esc>u
	while( buffer[indx] < '0' || buffer[indx] > '9' )
		indx++;
	
	// Skip CID
	indx++;
	
	// Copy source IP
	while( buffer[indx] != ' ' )
		host[indx2++] = buffer[indx++];
	host[indx2] = 0;	// Terminate
	
	// Scan up to data
	while( buffer[indx] != '\t' )
		indx++;
	
	// Skip tab
	indx++;
	
	// Copy message
	indx2 = 0;
	while( buffer[indx] != 27 && buffer[indx] != 0 )
		msg[indx2++] = buffer[indx++];
	msg[indx2] = 0;	// Terminate
	
	if( strncmp_P( msg, PSTR("LEDON"), 5 ) == 0 )
		// LED ON
		DEBUG_PORT |= (1<< DEBUG_BIT);
	
	if( strncmp_P( msg, PSTR("LEDOFF"), 6 ) == 0 )
		// LED OFF
		DEBUG_PORT &= ~(1<< DEBUG_BIT);
	
	if( strncmp_P( msg, PSTR("LED"), 3 ) == 0 && msg[3] >= '0' && msg[3] <= '9' )
	{
		indx2 = msg[3] - '0';
		for( indx = 0; indx < indx2; indx++ )
		{
			DEBUG_PORT |= (1<< DEBUG_BIT);
			_delay_ms( 100 );
			DEBUG_PORT &= ~(1<< DEBUG_BIT);
			_delay_ms( 100 );
		}
	}
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
	
	DDRB |= (1<< PB4);	/// TEMP: use PB4 as GND for debugging LED
	PORTB &= ~(1<< PB4);
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
	int step;
	
	// Initialize
	setup_ports();
	setup_serial();
	setup_interrupts();
	sei();
	
	// Wait a bit to let the WiFi module power up
	_delay_ms( 1000 );
	DEBUG_PORT |= (1<< DEBUG_BIT);
	_delay_ms( 1000 );
	DEBUG_PORT &= ~(1<< DEBUG_BIT);
	_delay_ms( 1000 );


	/*
	 * AT					// Are we live?
	 * AT+WM=1				// Mode=ad hoc
	 * AT+DHCPSRVR=1		// Enable DHCP server
	 * AT+WAUTH=2			// WEP security
	 * AT+WWEP1=616c69656e	// passphrase = 'alien'
	 * AT+WA=GS1011M		// Create ad hoc network named 'GS1011M'
	 * AT+MCSTSET=1			// enable multicast reception
	 * AT+NSUDP=49000		// Listen on UDP port 49000
	 */

	// Let's see if it works at all
	step = 1;
	if( !send_serial_command_P( PSTR("AT\r") ) == S2W_SUCCESS )
		goto fail;

	step = 2;
	if( !send_serial_command_P( PSTR("AT+WM=1\r") ) == S2W_SUCCESS )
		goto fail;
	
	step = 3;
	if( !send_serial_command_P( PSTR("AT+DHCPSRVR=1\r") ) == S2W_SUCCESS )
		goto fail;
	
	step = 4;
	if( !send_serial_command_P( PSTR("AT+WAUTH=2\r") ) == S2W_SUCCESS )
		goto fail;
	
	step = 5;
	if( !send_serial_command_P( PSTR("AT+WWEP1=616c69656e\r") ) == S2W_SUCCESS )
		goto fail;
	
	step = 6;
	if( !send_serial_command_P( PSTR("AT+WA=GS1011M\r") ) == S2W_SUCCESS )
		goto fail;
	
	step = 7;
	if( !send_serial_command_P( PSTR("AT+MCSTSET=1\r") ) == S2W_SUCCESS )
		goto fail;

	step = 8;
	if( !send_serial_command_P( PSTR("AT+NSUDP=49000\r") ) == S2W_SUCCESS )
		goto fail;
	
	// Set UDP receive handler
	udpHandler = &receiveUdp;
	
fail:
	debug_error( step );
	
	// Main loop
	while(1)
	{
		
	}

    return 0;
}
