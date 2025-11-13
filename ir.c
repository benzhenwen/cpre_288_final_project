#include "ir.h"
#include "timer.h"
#include <math.h>

#include <inc/tm4c123gh6pm.h>


// init the ir scanner
void ir_init_fuck() {
    // Enable clocks
    SYSCTL_RCGCGPIO_R |= 0x2;   // Port B
    SYSCTL_RCGCADC_R |= 0x1;    // ADC0
    timer_waitMillis(1);

    // Configure PB4 for analog function (AIN10)
    GPIO_PORTB_DIR_R   &= ~0x10; // PB4 input
    GPIO_PORTB_AFSEL_R |=  0x10; // PB4 uses alternate function (ADC)
    GPIO_PORTB_DEN_R   &= ~0x10; // disable digital on PB4
    GPIO_PORTB_AMSEL_R |=  0x10; // enable analog mode on PB4


    // Hardware averaging
    // SAC: 0=none, 1=2x, 2=4x, 3=8x, 4=16x, 5=32x, 6=64x
    ADC0_SAC_R = 0x04;                // 16x average

    // Disable SS0 while configuring
    ADC0_ACTSS_R &= ~0x1;

    // Trigger source: continuous (0xF)
    ADC0_EMUX_R = (ADC0_EMUX_R | 0x000F);

    // Multiplexer: SS0 sample 0 reads AIN10 (PB4)
    // Each slot is 4 bits in SSMUX0
    ADC0_SSMUX0_R = (ADC0_SSMUX0_R & ~0x000F) | 0xA;  // MUX0 = 10 (AIN10)

    // Sample control: mark end of sequence
    ADC0_SSCTL0_R = 0b0110;

    // Clear any prior interrupts and unmask if you want to use NVIC
    ADC0_ISC_R  = 0x01;       // clear SS0 flag
    ADC0_IM_R  &= ~(1U << 0); // mask SS0 interrupt

    // Re-enable SS0
    ADC0_ACTSS_R |= 0x1;
}


int ir_read_sample_fuck() {
    return ADC0_SSFIFO0_R & 0x0FFF;
}

int ir_floor_sample() {
    int min_value = ir_read_sample_fuck();

    int i;
    for (i = 1; i < 8; i++) {
        timer_waitMillis(2);
        int new_value = ir_read_sample_fuck();
        if (new_value < min_value) min_value = new_value;
    }

    return min_value;
}

float a = 0;
float b = 0;

void ir_set_a_b(float v_a, float v_b) {
    a = v_a;
    b = v_b;
}

float ir_raw_to_cm(int ir_raw_sample) {
    return a/ir_raw_sample + b;
}






// automatic calibration
float sum_n;  // num points
float sum_t;  // (1/ir)
float sum_y;  // (dist)
float sum_t2; // (1/ir)^2
float sum_ty; // (1/ir)*dist

void ir_auto_cal_init() {
    sum_n = 0;
    sum_t = 0;
    sum_y = 0;
    sum_t2 = 0;
    sum_ty = 0;
}

void ir_auto_cal_add_point(float ir_value, float distance) {
    if (ir_value == 0) return;

    float t = 1.0f / ir_value;
    float y = distance;

    sum_n++;
    sum_t += t;
    sum_y += y;
    sum_t2 += t*t;
    sum_ty += t*y;
}

Curve ir_auto_cal_calculate() {
    double denom = sum_n * sum_t2 - sum_t * sum_t;
    if (denom == 0) return (Curve) {0, 0};

    double a_hat = (sum_n * sum_ty - sum_t * sum_y) / denom;
    double b_hat = (sum_y - a_hat * sum_t) / sum_n;

    return (Curve) {a_hat, b_hat};
}




