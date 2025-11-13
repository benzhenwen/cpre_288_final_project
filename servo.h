#pragma once

// init the servo, starts the pwm timer immediately
void sv_init();

// set the high width of the wave in ticks. 16000 per ms.
void sv_set_width(uint32_t width_ticks);

// sets the servo angle, blocking delay included until servo is predicted to be in place
void sv_set_angle(int angle);

// blocking calibration routine
void sv_cal();

// cal known
void sv_set_cal_known(uint32_t min_val, uint32_t max_val);
