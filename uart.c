#include "uart.h"

#include <inc/tm4c123gh6pm.h>

#include "timer.h"

// basic init
void ur_init() {

    // ENABLE COMPONENTS
    SYSCTL_RCGCGPIO_R |= 0x02; // enable GPIO B
    SYSCTL_RCGCUART_R |= 0x02; // enable UART1 clock
    timer_waitMillis(1); // Small delay

    // GPIO CONF
    GPIO_PORTB_DEN_R |= 0x03;   // enable pins B0 and B1
    GPIO_PORTB_AFSEL_R |= 0x03; // afsel enable pins B0 and B1

    GPIO_PORTB_PCTL_R |= 0x11;  // force B0, B1 to value 0x01
    GPIO_PORTB_PCTL_R &= ~0xEE; // ^^^

    GPIO_PORTB_DIR_R &= ~0x01; // pin B0 (U1Rx) as input
    GPIO_PORTB_DIR_R |= 0x02;  // pin B1 (U1Rx) as output

    // UART CONFIG
    UART1_CTL_R &= ~0x01; // disable UART1

    UART1_IBRD_R = 0x08; // config baud rate (115200 for WiFi)
    UART1_FBRD_R = 0x2C;

    // Line Control Config - 0b0_11_0_0_0_0_0  no partiy _ 8 bit word (2 bits) _ no FIFOs _ 1 stop bit _ no parity _ no parity _ no send break
    UART1_LCRH_R = 0x60; // 0b01100000
    UART1_CC_R = 0x00;   // use system clock as source

    UART1_CTL_R |= 0x01; // enable UART1

}

// get byte basic wrapper
inline char ur_get_byte() {
    while(UART1_FR_R & 0x10) /* noop */; // waits until flag register bit 4 is cleared (when the receive FIFO is empty)
    return UART1_DR_R & 0xFF;            // get data and mask
}

// send byte basic wrapper
void ur_send_byte(char c) {
    while(UART1_FR_R & 0x20) /* noop */; // waits until flag register bit 5 is cleared (when the transmit FIFO is empty)
    UART1_DR_R = c;                      // set the data register
}


// more useful stuff
void ur_send_string(char * str) {
    while (*str != '\0') {
        ur_send_byte(*str);
        str++;
    }
}

void ur_send_line(char * str) {
    ur_send_string(str);
    ur_send_byte('\n');
}

// Send a 32-bit float in little-endian byte order over UART.
void ur_send_float(float f) {
    // Device is little-endian; transmit raw bytes as-is.
    uint8_t b[4];
    memcpy(b, &f, sizeof(b));
    ur_send_byte((char)b[0]);
    ur_send_byte((char)b[1]);
    ur_send_byte((char)b[2]);
    ur_send_byte((char)b[3]);
}




// interrupts
volatile char interrupt_buffer[64];
volatile int interrupt_buffer_c = 0;
volatile char interrupt_buffer_line_ready_flag = 0;

char ur_intr_line_ready() {
    return interrupt_buffer_line_ready_flag;
}
char * ur_intr_get_line() {
    interrupt_buffer_c = 0;
    interrupt_buffer_line_ready_flag = 0;
    return interrupt_buffer;
}


#include "lcd.h"

// interrupt callback
void ur_inter_handle() {
    if (!(UART1_MIS_R & 0x10)) return; // check if interrupt status is recieve raw

    char c = UART1_DR_R & 0xFF; // get data and mask
    interrupt_buffer[interrupt_buffer_c++] = c;

    if (c == '\n') {
        interrupt_buffer[interrupt_buffer_c - 1] = '\0';
        interrupt_buffer_line_ready_flag = 1;
    }

    UART1_ICR_R |= 0x10; // clear the interrupt
}

// interrupt init - must call ur_init first!
void ur_inter_init() {
    UART1_IM_R |= 0x10;    // enable bit 4 - recieve interrupt mask
    NVIC_EN0_R |= 1 << 6; // enable NVIC UART 1 (interrupt number 6)
    IntRegister(INT_UART1, ur_inter_handle);
}
