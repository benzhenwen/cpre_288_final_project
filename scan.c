#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "scan.h"
#include "Timer.h"

#include "uart.h"
#include "ir.h"
#include "ping.h"
#include "servo.h"



// point the servo
void sc_point_servo(int angle) {
    sv_set_angle(angle);
}



// sweep scan with ping sensor, 2 deg increment. populates float array of minimum length 91.
void sc_sweep_sound(float output[180 / SCAN_RESOLUTION + 1]) {
    int i;
    for (i = 0; i <= 180; i += SCAN_RESOLUTION) {
        sc_point_servo(i);
        output[i / SCAN_RESOLUTION] = pb_get_dist();
    }
}

float sc_scan_sound(int angle) {
    sc_point_servo(angle);
    return pb_get_dist();
}

// sweep scan with ir sensor, 2 deg increment. populates int array of minimum length 91 with RAW VALUES.
void sc_sweep_ir(float output[180 / SCAN_RESOLUTION + 1]) {
    int i;
    for (i = 0; i <= 180; i += SCAN_RESOLUTION) {
        sv_set_angle(i);
        int ir_raw_val = ir_floor_sample();
        output[i / SCAN_RESOLUTION] = ir_raw_to_cm(ir_raw_val);
    }
}


// get an individual ir scan value, raw
// angle input 0-180
int sc_scan_ir(int angle) {
    sc_point_servo(angle);
    return ir_floor_sample();
}

// print sweep scan
void sc_print_sweep(float * data, int data_c) {
    ur_send_line("Angle(Degrees)   Distance(m)");

    int i;
    for (i = 0; i < data_c; i++) {
        char buff[64];
        sprintf(buff, "%-16d%.1f", i * SCAN_RESOLUTION, data[i]);
        ur_send_line(buff);
    }
}

// print sweep scan with int values, assumes 2 deg increment
void sc_print_sweep_raw(int * data, int data_c) {
    ur_send_line("Angle(Degrees)   raw");

    int i;
    for (i = 0; i < data_c; i++) {
        char buff[64];
        sprintf(buff, "%-17d%d", i * SCAN_RESOLUTION, data[i]);
        ur_send_line(buff);
    }
}



// clean up a scan
void sc_clean_scan(float * data, int data_c) {
    int sweep_range = 4;
    while (sweep_range >= 3) { // multipass,
        int i;
        for (i = 0; i < data_c - sweep_range; i++) {

            float edge_far = data[i];
            float edge_close = data[i + sweep_range - 1];

            if (edge_far < edge_close) { // swap if close is further than far
                float temp = edge_far;
                edge_far = edge_close;
                edge_close = temp;
            }

            int j;
            for (j = 1; j < sweep_range - 1; j++) { // for each of the middle values, if they are too far out of range adjust them
                if (data[i + j] > edge_far + 2) data[i + j] = edge_far;
                if (data[i + j] < edge_close - 2) data[i + j] = edge_close;
            }

        }

        sweep_range--;
    }
}




// pass the scan data (ideally after sc_clean_scan was called)
// populates objects and objects_c, finding objects within the max distance
// filters objects with a rad smaller than min_rad
#define MARCH_EDGE_TOLERANCE 10

// a compensation of 3 degrees (clockwise) due to scan sweep latency
#define SWEEP_ANGLE_COMP (-5)


void sc_find_objects(float * data, int data_c, float max_distance, int min_rad, object_radial * objects, int * objects_c) { 
    float march_min = data[0]; // we follow the line, allowing it to expand slowly for "diagonal" objects
    float march_max = data[0];

    int march_start = 0; // the start of the march in deg

    int i;
    for (i = 1; i < data_c; i++) {
        if (data[i] > march_max + MARCH_EDGE_TOLERANCE || data[i] < march_min - MARCH_EDGE_TOLERANCE || i == data_c - 1) { // next step not within 5 units or end of scan, end march
            int march_length = i - march_start;
            float march_dist = (march_min + march_max) / 2; // innacurate, but good enough approx of object dist (for now)

            if (
                    march_length >= (min_rad / SCAN_RESOLUTION) && march_dist <= max_distance && // found an object within our constraints
                    march_start > (2 / SCAN_RESOLUTION) && (march_start + march_length) < (180 - (2 / SCAN_RESOLUTION)) // object isn't on the very edge of the scan
            ) {
                objects[*objects_c].angle = (march_start + (march_length / 2)) * SCAN_RESOLUTION + SWEEP_ANGLE_COMP;
                objects[*objects_c].diameter = march_length * SCAN_RESOLUTION;
                objects[*objects_c].distance = march_dist;
                
                (*objects_c)++;
            }

            if (i != data_c - 1) { // reset for new march
                march_min = data[i+1];
                march_max = data[i+1];
                march_start = i;
            }
        } else {
            if      (data[i] > march_max) march_max = data[i]; // expand the scope of the march
            else if (data[i] < march_min) march_min = data[i];
        }
    }

}

// prints out the data from sc_find_objects
void sc_print_objects(object_radial * objects, int objects_c) {
    ur_send_line("Object  Angle  Distance  angluar  size");

    int i;
    for (i = 0; i < objects_c; i++) {
        char buff[64];
        sprintf(buff, "%-8d%-7d%-10.2f%-10d%.2f", i, objects[i].angle, objects[i].distance, objects[i].diameter, objects[i].size);
        ur_send_line(buff);
    }
}



// refinds the distances to objects with the ping sensor
void sc_reping_objects(object_radial * objects, int objects_c) {
    int i;
    for (i = 0; i < objects_c; i++) {

        // point
        sc_point_servo(objects[i].angle);

        // get accurate distance measurement
        float dist = sc_scan_sound(objects[i].angle);

        // update the object
        objects[i].distance = dist;

    }
}

// populates the size property of the objects
// requires the angular radius and distance to be accurate. it's recommended to use an ir sweep scan and then reping objects before calling this.
void sc_calc_size_objects(object_radial * objects, int objects_c) {
    int i;
    for (i = 0; i < objects_c; i++) {

        // update the object
        objects[i].size = objects[i].distance * (objects[i].diameter * M_PI) / 180.0f; // rough approximation

    }
}








