#pragma once



#include "main_scan_data.h"
#include "scan.h"

void update_object_map();


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

    // updates the object map
    update_object_map();
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





// object map stuff
void remove_object_from_map(int index) {
    int i;
    for (i = index; i < object_map_c - 1; i++) {
        object_map[i] = object_map[i+1]; // left shift to fill the removed space
    }
    object_map_c--;
}

void add_wall_object(float x, float y, float r) {
    // remove all now irrelevant walls
    int i;
    for (i = 0; i < object_map_c; i++) {

        // basic info about the object
        const float dy = object_map[i].y - y;
        const float dx = object_map[i].x - x;

        const float dist_bearing = sqrtf(dx*dx + dy*dy);

        // border (white) objects dissapear when another one roughly takes its place
        if (object_map[i].type == 3) {

            if (dist_bearing < r) {
                ur_send_line("removing border cos it's a stupid little bitch");
                remove_object_from_map(i);
                i--;
            }
        }
    }

    // add the wall
    object_map[object_map_c] = (object_positional) { x, y, r, (char) (3) };
    object_map_c++;
}

void update_object_map() {

    // remove all now irrelevant objects
    int i;
    for (i = 0; i < object_map_c; i++) {

        // basic info about the object
        const float dy = object_map[i].y - get_pos_y();
        const float dx = object_map[i].x - get_pos_x();

        const float dist_bearing = sqrtf(dx*dx + dy*dy);

        // tall objects disappear when we re-scan over their cone
        if (object_map[i].type == 1) {
            // calculate bearing and distance from robot

            float angle_bearing = atan2f(dy, dx) * (180 / M_PI) - get_pos_r();
            if (angle_bearing >  180) angle_bearing -= 360;
            if (angle_bearing < -180) angle_bearing += 360;

            char buff[32];
            sprintf(buff, "object check removal: %.2f, %.2f", dx, dy);
            ur_send_line(buff);

            if (angle_bearing > -70 && angle_bearing < 70 && dist_bearing < SCAN_MAX_DISTANCE*10) {
                ur_send_line("removing object cos it's a stupid little bitch");
                remove_object_from_map(i);
                i--;
            }
        }
    }

    // add the newly scanned objects
    for (i = 0; i < objects_c; i++) {
        // object rel pos
        float tx = (objects[i].distance + objects[i].size / 2.0f) * 10 * cosf((objects[i].angle - 90 + get_pos_r()) * (M_PI / 180));
        float ty = (objects[i].distance + objects[i].size / 2.0f) * 10 * sinf((objects[i].angle - 90 + get_pos_r()) * (M_PI / 180));

        // scanner offset
        tx += 90 * cosf(get_pos_r() * (M_PI / 180));
        ty += 90 * sinf(get_pos_r() * (M_PI / 180));

        object_map[object_map_c] = (object_positional) { get_pos_x() + tx, get_pos_y() + ty, objects[i].size * 10 / 2, (char) 1 };
        object_map_c++;
    }
}
