#include "ping.h"
#include "Timer.h"

#include <inc/tm4c123gh6pm.h>






// the interrupt handler for the ping timer (3b)
volatile unsigned int rising_edge_time;
volatile unsigned int falling_edge_time;

volatile unsigned char pulse_flag; // 0 = reading first input, 1 = reading second input
volatile unsigned char read_ready_flag = 0;

void pn_interrupt_handle() {
    // check for correct interrupt, which is catpure event
    if (! (TIMER3_MIS_R & 0x0400)) return;

    // capture the value
    unsigned int capture_value = TIMER3_TBR_R & 0x00FFFFFF;

    // store it
    if (!pulse_flag) {
        rising_edge_time = capture_value;
        pulse_flag = 1;
    } else {
        falling_edge_time = capture_value;
        read_ready_flag = 1;
    }

    // clear the interrupt
    TIMER3_ICR_R |= 0x0400;
}

unsigned int read_recent_pulse_length() {
    if (rising_edge_time > falling_edge_time) {
        return rising_edge_time - falling_edge_time;
    }
    return rising_edge_time - falling_edge_time + 0xFFFFFF;
}







/*
 * prelab 9
 * a) timer3 regs:
 * CTL
 * MCFG
 * TBMR
 * IMR / MIS / ICR
 * TBIL
 * TBP
 *
 * 2) from delta to distance
 * delta / 2 * ((34000 cm/s) / (16000000 hz)) = delta * 0.0010625
 *
 * 3) GPTM Raw Interrupt does not account for the mask the user applies. we used GPTMMIS in the interrupt handler
 */







void pn_init() {
    // notes: ping sensor on PB3
    //        using timer 3_tb + prescaler

    // enable clocks
    SYSCTL_RCGCGPIO_R |= 0x2;   // Port B
    SYSCTL_RCGCTIMER_R |= 0x8;  // timer 3
    timer_waitMillis(1);

    // gpio config (pin PB3 to T3CCP1)
    GPIO_PORTB_DIR_R   |=  0x8;                                   // PB3 output (for now!)
    GPIO_PORTB_AFSEL_R &= ~0x8;                                   // PB3 uses software output (for now!)

    // for later, this can be configured once and then swapping AFSEL will auto toggle to the proper device
    GPIO_PORTB_PCTL_R   = (GPIO_PORTB_PCTL_R & ~0xF000) | 0x7000; // alternate function set to T3CCP1

    /* later on...
    GPIO_PORTB_DIR_R   &= ~0x8;                                   // PB3 input
    GPIO_PORTB_AFSEL_R |=  0x8;                                   // PB3 uses alternate function
    */

    GPIO_PORTB_DEN_R   |=  0x8;                                   // enable digital on PB3
    GPIO_PORTB_AMSEL_R &= ~0x8;                                   // disable analog mode on PB3

    // timer config
    TIMER3_CTL_R &= ~0x0100;                       // disable timer 3B
    TIMER3_CFG_R = (TIMER3_CFG_R & ~0b0111) | 0x4; // timer on split mode (just using b)

    /*
     * [1:0] - mode set to capture (0b11)
     * [2]   - capture sub mode to edge-time (0b1)
     * [3]   - alternate mode on capture/compare, not pwm (0b0)
     * [4]   - count dir down (0b0)
     * [5]   - no interrupt on match (0b0)
     * [6]   - start counting as soon as enabled (0b0)
     * [7]   - write snapshot into GPTMTBR (0b1)
     * [8]   - update ILR/PR on current cycle (0b0)
     * [9]   - bit not valid because not in PWM mode (0b0)
     * [10]  - update GPTMTBMATCHR and GPTMTBPR on next cycle (0b0)
     * [11]  - leave low, legacy PWM option (0b0)
     */
    TIMER3_TBMR_R = (TIMER3_TBMR_R & 0x0FFF) | 0b000010000111;

    /*
     * [15]    - reserved (0b0)
     * [14]    - not using pwm, no invert (0b0)
     * [13]    - no ADC trigger (0b0)
     * [12]    - reserved (0b0)
     * [11:10] - timer event mode on both edges (0b11)
     * [9]     - timer stall disabled (0b0)
     * [8]     - keep disabled (0b0)
     * [7:0]   - not relevant to timer B (0x00)
     */
    TIMER3_CTL_R = (TIMER3_CTL_R & ~0b0110111101111111) | 0b0000110000000000;

    // config interrupt
    TIMER3_IMR_R  |= 0x0400;    // enable bit 10 - capture mode event interrupt (so on both edges of clk)
    NVIC_EN1_R |= 1 << (36 - 32); // enable 16/32-Bit Timer 3B (interupt number 36)
    IntRegister(INT_TIMER3B, pn_interrupt_handle);

    // re-enable timer
    TIMER3_CTL_R |= 0x0100; // enable timer 3B
}




void pb_send_ping() {
    // set gpio pin as output
    GPIO_PORTB_DIR_R   |=  0x8;
    GPIO_PORTB_AFSEL_R &= ~0x8;

    // mask interrupt so we don't trigger it on manual ping
    TIMER3_IMR_R &= ~0x0400;

    // prime the echo pulse
    GPIO_PORTB_DATA_R |= 0x08; // set pin B3

    // timer start
    TIMER3_TBILR_R |= 0xFFFF; // load max for the starting value of the timer
    TIMER3_TBPR_R |= 0xFF; // load the max for the starting value of the prescale timer

    // clear the trigger pulse
    timer_waitMicros(5); // wait the 5us send trigger min
    GPIO_PORTB_DATA_R &= ~0x08; // clear pin B3

    // set gpio pin as input afsel to timer
    GPIO_PORTB_DIR_R   &= ~0x8;
    GPIO_PORTB_AFSEL_R |=  0x8;

    // allow for interrupt to be received
    TIMER3_IMR_R |= 0x0400;

    // clear pulse flag to prepare for reading
    read_ready_flag = 0;
    pulse_flag = 0;
}

float pb_get_dist() {

    // send off the ping
    pb_send_ping();

    // wait for the ping to be ready
    while (!read_ready_flag) /* noop */ ;

    return ((float) read_recent_pulse_length()) * 0.0010625f;
}

















