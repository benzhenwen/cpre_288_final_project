#pragma once


#include "main_scan_data.h"

#include "movement.h"
#include "command.h"
#include "movementcommands.h"



char full_auto_flag = 0; // flag that tries to repath on fail

char move_bump_interrupt_callback(oi_t * sensor_data);

#define APPROACH_TOLERANCE 5
void do_object_scan_and_approach(CommandData * data) {
    perform_scan_and_obj_detection();
    update_object_map();
    send_data_packet(object_map, object_map_c); // update python data packet

    // move to smallest
    int smallest_index = find_smallest_object_index();
    if (smallest_index != -1) {
        // error is bot size + smallest object size
        cq_queue(gen_approach_cmd_intr(object_map[smallest_index].x, object_map[smallest_index].y, 160 + APPROACH_TOLERANCE + (object_map[smallest_index].radius), &move_bump_interrupt_callback));
    }
}

char identify_ground_object_interrupt_callback(oi_t * sensor_data) {
    if (!(sensor_data->bumpLeft && sensor_data->bumpRight)) return 0;

    move_stop();
    ur_send_line("identified ground object");

    float tx = (160 + 65) * cosf(get_pos_r() * (M_PI / 180));
    float ty = (160 + 65) * sinf(get_pos_r() * (M_PI / 180));
    object_map[object_map_c] = (object_positional) { get_pos_x() + tx, get_pos_y() + ty, 65, (char) 0 };
    object_map_c++;

    send_data_packet(object_map, object_map_c); // update python data packet

    if (full_auto_flag) {
        cq_queue(gen_move_reverse_cmd(300));

        float mid_x;
        float mid_y;
        int target_object_map_index = find_smallest_object_index();

        float predicted_start_x = get_pos_x() - cosf(get_pos_r() * (M_PI / 180)) * 300;
        float predicted_start_y = get_pos_y() - sinf(get_pos_r() * (M_PI / 180)) * 300;
        find_valid_approach(predicted_start_x, predicted_start_y, object_map[target_object_map_index].x, object_map[target_object_map_index].y, 400, 700, object_map[target_object_map_index].radius, &mid_x, &mid_y);

        // to midpoint and approach
        cq_queue(gen_move_to_cmd_intr(mid_x, mid_y, &move_bump_interrupt_callback)); // error is bot size + smallest object size
        cq_queue(gen_approach_cmd_intr(object_map[target_object_map_index].x, object_map[target_object_map_index].y, 400, &move_bump_interrupt_callback)); // error is bot size + smallest object size
        cq_queue(gen_rotate_to_cmd(atan2f(object_map[target_object_map_index].y - mid_y, object_map[target_object_map_index].x - mid_x) * 180 / M_PI));

        // rescan and go
        cq_queue((Command) {&do_object_scan_and_approach, &always_true, &always_false, (CommandData) {0} });
    }

    return 1;
}

char move_bump_interrupt_callback(oi_t * sensor_data) {
    if (!(sensor_data->bumpLeft || sensor_data->bumpRight)) return 0;

    move_stop();
    ur_send_line("bumped");

    cq_queue(gen_rotate_cmd_intr(sensor_data->bumpRight ? -90 : 90, &identify_ground_object_interrupt_callback));

    return 1;
}
