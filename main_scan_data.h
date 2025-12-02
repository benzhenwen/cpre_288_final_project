#pragma once




// 1 degree increments, 181 data points
#define SCAN_BUFFER_SIZE (180 / SCAN_RESOLUTION + 1)

// only allow objects up to x cm away
#define SCAN_MAX_DISTANCE 70

// scan data buffers
float data[SCAN_BUFFER_SIZE];

// object radial storage (small temp)
object_radial objects[8];
int objects_c;

// object map (xy based)
object_positional object_map[64];
int object_map_c;


