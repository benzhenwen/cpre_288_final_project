#include "Timer.h"
#include "button.h"
#include "lcd.h"

#include <stdint.h>
#include <math.h>
#include <inc/tm4c123gh6pm.h>

#include "servo.h"



#define SERVO_PERIOD_TICKS       (320000u)                 // 20 ms * 16 MHz
#define SERVO_LOAD_24            (SERVO_PERIOD_TICKS - 1u) // 319,999 = 0x04E1FF

static volatile uint32_t g_servo_high_ticks = 24000; // default value is ~50%

void sv_init() {
    SYSCTL_RCGCGPIO_R |= 0x02; // clock timer 1
    SYSCTL_RCGCTIMER_R |= 0x02; // click GPIO PB
    timer_waitMillis(1);

    // GPIO: PB5 as T1CCP1
    GPIO_PORTB_AFSEL_R |= (1 << 5); // AF
    GPIO_PORTB_DEN_R   |= (1 << 5); // DEN
    // PCTL: set PB5 nibble to 0x7 (T1CCP1)
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & ~((0xF) << (5 * 4))) | ((0b0111) << (5 * 4));

    // Timer1B PWM @ 20 ms, non-inverted, down count
    TIMER1_CTL_R  &= ~(1u << 8); // EN B = 0
    TIMER1_CFG_R   = 0x4;        // split mode

    /* TBMR:
       - TnMR=0b10 (periodic)
       - TnAMS=1 (PWM mode)
       - TnCDIR=0 (down)
       - TnMRSU=1 (bit10): update MATCH on next cycle (glitchless updates) */
    TIMER1_TBMR_R  = 0;
    TIMER1_TBMR_R |= 0x2u;       // periodic
    TIMER1_TBMR_R |= (1u << 3);  // AMS=1 -> PWM
    TIMER1_TBMR_R &= ~(1u << 4); // down-count
    TIMER1_TBMR_R |= (1u << 10); // MRSU: match updates take effect at rollover

    // Non-inverted PWM output: TBPWML=0 (output set at reload, cleared at match)
    TIMER1_CTL_R  &= ~(1u << 14);

    // Period = 0x04E1FF -> TBPR=0x04, TBILR=0xE1FF
    TIMER1_TBPR_R  = (uint8_t)((SERVO_LOAD_24 >> 16) & 0xFFu);
    TIMER1_TBILR_R = (uint16_t)(SERVO_LOAD_24 & 0xFFFFu);

    // Initial high-time = 1.5 ms -> 24,000 ticks -> MATCH = LOAD - H
    uint32_t match_init = SERVO_LOAD_24 - g_servo_high_ticks;
    TIMER1_TBPMR_R   = (uint8_t)((match_init >> 16) & 0xFF);
    TIMER1_TBMATCHR_R= (uint16_t)(match_init & 0xFFFF);

    TIMER1_CTL_R  |= (1u << 8); // TBEN=1 */
}



/* Sets the PWM HIGH width in raw clock cycles (ticks).
   Example: 1.5 ms -> 0.0015 * 16e6 = 24000 ticks. */
void sv_set_width(uint32_t width_ticks)
{
    if (width_ticks > SERVO_LOAD_24) {
        width_ticks = SERVO_LOAD_24;
    } else if (width_ticks == 0) {
        width_ticks = 1;
    }

    g_servo_high_ticks = width_ticks;

    uint32_t match = SERVO_LOAD_24 - g_servo_high_ticks;
    TIMER1_TBPMR_R    = (uint8_t)((match >> 16) & 0xFF);
    TIMER1_TBMATCHR_R = (uint16_t)(match & 0xFFFF);
}





// calibration stuff
static uint32_t SERVO_MIN_VALUE = 12000;
static uint32_t SERVO_MAX_VALUE = 36000;

void sv_set_cal_known(uint32_t min_val, uint32_t max_val) {
    SERVO_MIN_VALUE = min_val;
    SERVO_MAX_VALUE = max_val;
}

void sv_set_angle(int angle) {
    const int new_servo_value = SERVO_MIN_VALUE + roundf( (angle / 180.0f) * (SERVO_MAX_VALUE - SERVO_MIN_VALUE) );
    unsigned int wait_time = (abs(new_servo_value - g_servo_high_ticks) * 800u) / (SERVO_MAX_VALUE - SERVO_MIN_VALUE); // rotation speed of about n milli seconds per 180 deg

    sv_set_width(new_servo_value);
    timer_waitMillis(wait_time);
}

// blocking calibration function call
void sv_cal() {
    button_init();
    lcd_printf("b1: left - b2: rightb4: exit\nraw val: %d", g_servo_high_ticks);

    while(1) {
        if (button_getButtons() & 0b1000) break; // break condition

        char new_print = 0;

        if (button_getButtons() & 0b0001) {
            sv_set_width(g_servo_high_ticks + 150);
            new_print = 1;
        }
        if (button_getButtons() & 0b0010) {
            sv_set_width(g_servo_high_ticks - 150);
            new_print = 1;
        }

        if (new_print) {
            lcd_printf("b1: left - b2: rightb4: exit\nraw val: %d", g_servo_high_ticks);
            timer_waitMillis(10);
        }
    }

}

