#pragma once



#include "main_scan_data.h"
#include "scan.h"




void perform_scan_and_obj_detection() {
    objects_c = 0;

    ur_send_line("scanning...");


    // gather data
    sc_sweep_ir(data);

    sc_clean_scan(data, SCAN_BUFFER_SIZE);
    sc_print_sweep(data, SCAN_BUFFER_SIZE);

    // convert to objects
    sc_find_objects(data, SCAN_BUFFER_SIZE, SCAN_MAX_DISTANCE, 4, objects, &objects_c);


    // no objects
    if (objects_c == 0) {
        ur_send_line("no objects found");
        return;
    }

    // get better object data
    sc_reping_objects(objects, objects_c);

    // populate object size values
    sc_calc_size_objects(objects, objects_c);

    // print out the objects
    sc_print_objects(objects, objects_c);
}

int find_smallest_object_index() {
    if (object_map_c == 0) {
        ur_send_line("Warning: tried calling find_smallest_object_index with no objects detected");
        return -1;
    }

    int i;
    int smallest_index = 0;
    float smallest_size = object_map[0].radius;
    for (i = 1; i < object_map_c; i++) {
        if (object_map[i].radius < smallest_size) {
            smallest_size = object_map[i].radius;
            smallest_index = i;
        }
    }
    return smallest_index;
}

void update_object_map() {
    int i;
    for (i = 0; i < objects_c; i++) {
        // object rel pos
        float tx = (objects[i].distance + objects[i].size / 2.0f) * 10 * cosf((objects[i].angle - 90 + get_pos_r()) * (M_PI / 180));
        float ty = (objects[i].distance + objects[i].size / 2.0f) * 10 * sinf((objects[i].angle - 90 + get_pos_r()) * (M_PI / 180));

        // scanner offset
        tx += 90 * cosf(get_pos_r() * (M_PI / 180));
        ty += 90 * sinf(get_pos_r() * (M_PI / 180));

        object_map[i] = (object_positional) { get_pos_x() + tx, get_pos_y() + ty, objects[i].size * 10 / 2, (char) 1 };
    }

    object_map_c = i;
}
