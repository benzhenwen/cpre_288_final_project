#pragma once

#define scan_point_data_t cyBOT_Scan_t
#define servo_cal_data_t cyBOT_SERVRO_cal_t



// SCAN RESOLUTION in deg
#define SCAN_RESOLUTION 1



typedef struct object_radial {
    int angle; // the angle of the center of the object
    int diameter; // the radial size of the object
    float distance; // the distace to the edge of the object
    float size; // the actual diameter of the object, in cm. need to call
} object_radial;


typedef struct object_positional {
    float x; // x pos in mm
    float y; // y pos in mm
    float radius; // radius of the object in mm;
    char type; // 0 is short object, 1 is tall object
} object_positional;


// runs init
void sc_init(int servo_enable, int ping_enable, int ir_enable);
// runs init with all values set to 1
void sc_init_all();



// point the servo
void sc_point_servo(int angle);

// runs a scan, returns a scan_data
void sc_sweep_sound(float * output);

// scan a single point with ping. returns dist
float sc_scan_sound(int angle);


// sweep scan with ir sensor, 2 deg increment. populates int array of minimum length 91 with RAW VALUES.
void sc_sweep_ir(float * output);

// get an individual ir scan value, raw
// angle input 0-180
int sc_scan_ir(int angle);



// pretty prints scan data to uart
void sc_print_sweep(float * data, int data_c);
// print sweep scan with int values, assumes 2 deg increment
void sc_print_sweep_raw(int * data, int data_c);

// clean up a scan
void sc_clean_scan(float * data, int data_c);


// pass the scan data (ideally after sc_clean_scan was called)
// populates objects and objects_c, finding objects within the max distance
// filters objects with a rad smaller than min_rad
void sc_find_objects(float * data, int data_c, float max_distance, int min_rad, object_radial * objects, int * objects_c);

// prints objects
void sc_print_objects(object_radial * objects, int objects_c);




// refinds the distances to objects with the ping sensor
void sc_reping_objects(object_radial * objects, int objects_c);

// populates the size property of the objects
// requires the angular radius and distance to be accurate. it's recommended to use an ir sweep scan and then reping objects before calling this.
void sc_calc_size_objects(object_radial * objects, int objects_c);

