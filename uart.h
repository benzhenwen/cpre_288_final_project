#pragma once


// basic init
void ur_init();

// get byte basic wrapper
char ur_get_byte();

// send byte basic wrapper
void ur_send_byte(char val);


// sends all bytes of a string pointer in order, terminating at the first \0
void ur_send_string(char * str);

// functions the ame as ur_send_string, but also sends \r and \n after
void ur_send_line(char * str);

// Send a 32-bit float in little-endian byte order over UART.
void ur_send_float(float f);


// uart interrupt init
void ur_inter_init();

// returns if the uart has a line ready to read. the line is a 64 char buffer that ends with a \0 assuming ur_intr_line_ready returns true
char ur_intr_line_ready();

// returns the character buffer
char * ur_intr_get_line();
