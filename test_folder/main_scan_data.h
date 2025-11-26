#pragma once



typedef struct object_positional {
    float x; // x pos in mm
    float y; // y pos in mm
    float radius; // radius of the object in mm;
    char type; // 0 is short object, 1 is tall object, 2 is hole (black), 3 is edge (white)
} object_positional;



// only allow objects up to x cm away
#define SCAN_MAX_DISTANCE 80

// object map (xy based)
object_positional object_map[64];
int object_map_c;


