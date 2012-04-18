//
//  main.h
//  wifitest
//
//  Created by Jens Willy Johannsen on 17-04-12.
//  Copyright (c) 2012 Greener Pastures. All rights reserved.
//

#ifndef wifitest_main_h
#define wifitest_main_h

// Status code definitions
#define S2W_PENDING 0	// pseudo-code – means "valid response code has not yet been received". Keep checking until status code != S2W_PENDING.
#define S2W_SUCCESS 1
#define S2W_FAILURE 2
#define S2W_EINVAL  3

// Prototypes
int send_serial_command_P( const prog_char *fmt, ... ) __attribute__((format(printf, 1, 2)));	// Let compiler check printf style format
void resetBuffer();
void receiveUdp( const char* buffer );

#endif
