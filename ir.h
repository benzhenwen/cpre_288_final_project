#pragma once

// init the ir scanner
// does continuous sampling with 16 hardware averages
void ir_init_fuck();

// reads an ir sample
int ir_read_sample_fuck();

// reads 16 ir values and returns the lowest
int ir_floor_sample();

// convert a raw ir sample to cm
float ir_raw_to_cm(int ir_raw_sample);


// call once to start an auto-calibration
void ir_auto_cal_init();

// add a data point to the auto-calibration. pass the raw ir value and the distance it should map to
void ir_auto_cal_add_point(float ir_value, float distance);

// after calling ir_auto_cal_add_point a number of times, this method will return the best fit values a and b for the curve y = a/x + b
typedef struct Curve {
    float a;
    float b;
} Curve;

Curve ir_auto_cal_calculate();

// set A and k based off of the autocal
void ir_set_a_b(float v_a, float v_b);
