#pragma once

#include "uart.h"
#include "movement.h"
#include "scan.h"

// send specialized message over uart that contains robot data for python

// Send the DATA header (no CR/LF), 6 floats (robot+target), objects, then 0x00 sentinel.
void send_data_packet(object_positional * object_map, int object_map_c, char do_objects) {
    // Header "DATA" (exactly 4 bytes, no newline)
    ur_send_byte('D');
    ur_send_byte('A');
    ur_send_byte('T');
    ur_send_byte('A');

    // Robot pos (mm, deg)
    ur_send_float(get_pos_x());
    ur_send_float(get_pos_y());
    ur_send_float(get_pos_r());

    // Target pos (mm, deg) 
    ur_send_float(get_target_x());
    ur_send_float(get_target_y());
    ur_send_float(get_target_r());

    // Target approach offset
    ur_send_float(get_target_apprach_distance_offset());

    // the move flag
    ur_send_byte(get_move_mode_flag());

    // send objects or not
    if (!do_objects) {
        ur_send_byte((unsigned char) 111);
        return;
    }

    // Total object count
    ur_send_byte((unsigned char) object_map_c);

    // objects: for each, send x, y, radius (float32 LE), then type (1 byte)
    int i;
    for (i = 0; i < object_map_c; i++) {
        const object_positional *o = &object_map[i];

        // sending data
        ur_send_float(o->x);
        ur_send_float(o->y);
        ur_send_float(o->radius);

        // type
        ur_send_byte(o->type);
    }
}

