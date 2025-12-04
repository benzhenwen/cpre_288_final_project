#pragma once


#include "main_scan_data.h"
#include "main_objects.h"

#include "movement.h"
#include "command.h"
#include "movementcommands.h"

#include <stdint.h>


char move_bump_interrupt_callback(oi_t * sensor_data);

//#define APPROACH_TOLERANCE 5
//void do_object_scan_and_approach(CommandData * data) {
//    perform_scan_and_obj_detection();
//    update_object_map();
//    send_data_packet(object_map, object_map_c, 1); // update python data packet
//
//    // move to smallest
//    int smallest_index = find_smallest_object_index();
//    if (smallest_index != -1) {
//        // error is bot size + smallest object size
//        cq_queue_front(gen_move_cmd(80));
//        (gen_approach_cmd_intr(object_map[smallest_index].x, object_map[smallest_index].y, 160 + APPROACH_TOLERANCE + (object_map[smallest_index].radius), &move_bump_interrupt_callback));
//    }
//}

char identify_ground_object_interrupt_callback(oi_t * sensor_data) {
    if (!(sensor_data->bumpLeft && sensor_data->bumpRight)) return 0;

    move_stop();
    ur_send_line("identified ground object");

    float tx = (160 + 65) * cosf(get_pos_r() * (M_PI / 180));
    float ty = (160 + 65) * sinf(get_pos_r() * (M_PI / 180));
    object_map[object_map_c] = (object_positional) { get_pos_x() + tx, get_pos_y() + ty, 65, (char) 0 };
    object_map_c++;

    send_data_packet(object_map, object_map_c, 1); // update python data packet

    cq_queue_front(gen_move_reverse_cmd(30));

    return 1;
}



// cliff detection

// white trigger was 2300 2500, black was 300

#define BLACK_THRESH 200
#define WHITE_THRESH_HIGH_TRIGGER 2750
#define WHITE_THRESH_LOW_TRIGGER 2600

// cliff type 0: normal, 1: black, 2: white for any of the cliff sensors



// flags
float cliff_detect_angle_a;
float cliff_detect_angle_b;

int cliff_turn_direction;
char cliff_type;

// the second callback for the cliff identification
char identify_cliff_interrupt_callback_2(oi_t * sensor_data) {
    const uint16_t fl_v = (sensor_data->cliffFrontLeftSignal);
    const uint16_t fr_v = (sensor_data->cliffFrontRightSignal);

    // flag value for each cliff - 0: normal, 1: black, 2: white
    const char fl_f = (fl_v < BLACK_THRESH) ? 1 : ((fl_v > WHITE_THRESH_LOW_TRIGGER) ? 2 : 0);
    const char fr_f = (fr_v < BLACK_THRESH) ? 1 : ((fr_v > WHITE_THRESH_LOW_TRIGGER) ? 2 : 0);

    // keep turning until both front sensors are no longer triggered
    if (fl_f || fr_f) return 0;

    // collect data point a
    cliff_detect_angle_b = get_pos_r();

    // add the cliff to the map
    if (cliff_detect_angle_b + 180 < cliff_detect_angle_a) cliff_detect_angle_b += 360;
    if (cliff_detect_angle_a + 180 < cliff_detect_angle_b) cliff_detect_angle_a += 360;
    float cliff_angle = ((cliff_detect_angle_a + cliff_detect_angle_b) / 2) * (M_PI / 180);

    const float cliff_object_rad = (cliff_type == 1) ? 100.0f : 32768.0f;

    const float tx = cosf(cliff_angle) * (cliff_object_rad + 160);
    const float ty = sinf(cliff_angle) * (cliff_object_rad + 160);

    if (cliff_type == 1) { // add normal hole
        object_map[object_map_c] = (object_positional) { get_pos_x() + tx, get_pos_y() + ty, cliff_object_rad + 50, (char) (2) };
        object_map_c++;
    } else { // wall object
        add_wall_object(get_pos_x() + tx, get_pos_y() + ty, cliff_object_rad);
    }

    send_data_packet(object_map, object_map_c, 1); // update python data packet

    // move away from the cliff a bit
    cq_queue_front(gen_move_cmd(50));
    cq_queue_front(gen_rotate_to_cmd(cliff_angle * (180 / M_PI) + 180));

    return 1;
}

// the point of this callback is simply to turn until no cliff sensors are being triggered
char identify_cliff_interrupt_callback(oi_t * sensor_data) {
    const uint16_t fl_v = (sensor_data->cliffFrontLeftSignal);
    const uint16_t fr_v = (sensor_data->cliffFrontRightSignal);

    // flag value for each cliff - 0: normal, 1: black, 2: white
    const char fl_f = (fl_v < BLACK_THRESH) ? 1 : ((fl_v > WHITE_THRESH_HIGH_TRIGGER) ? 2 : 0);
    const char fr_f = (fr_v < BLACK_THRESH) ? 1 : ((fr_v > WHITE_THRESH_HIGH_TRIGGER) ? 2 : 0);

    // keep turning until a front sensor triggers
    if (!(fl_f || fr_f)) return 0;

    // collect data point a
    cliff_detect_angle_a = get_pos_r();

    // keep rotating
    cq_queue_front(gen_rotate_cmd_intr(cliff_turn_direction, &identify_cliff_interrupt_callback_2));

    return 1;
}





char move_bump_interrupt_callback(oi_t * sensor_data) {

    // basic bump
    const char is_bump = sensor_data->bumpLeft || sensor_data->bumpRight;

    // basic cliff
    const uint16_t l_v  = (sensor_data->cliffLeftSignal);
    const uint16_t fl_v = (sensor_data->cliffFrontLeftSignal);
    const uint16_t fr_v = (sensor_data->cliffFrontRightSignal);
    const uint16_t r_v  = (sensor_data->cliffRightSignal);

    // flag value for each cliff - 0: normal, 1: black, 2: white
    const char l_f  = (l_v  < BLACK_THRESH) ? 1 : ((l_v  > WHITE_THRESH_HIGH_TRIGGER) ? 2 : 0);
    const char fl_f = (fl_v < BLACK_THRESH) ? 1 : ((fl_v > WHITE_THRESH_HIGH_TRIGGER) ? 2 : 0);
    const char fr_f = (fr_v < BLACK_THRESH) ? 1 : ((fr_v > WHITE_THRESH_HIGH_TRIGGER) ? 2 : 0);
    const char r_f  = (r_v  < BLACK_THRESH) ? 1 : ((r_v  > WHITE_THRESH_HIGH_TRIGGER) ? 2 : 0);

    const char is_cliff = l_f || fl_f || fr_f || r_f;

    // no interrupt check
    if (!is_bump && !is_cliff) return 0;

    move_stop();

    // bump handling
    if (is_bump) {
        ur_send_line("bump");
        cq_queue_front(gen_rotate_cmd_intr(sensor_data->bumpRight ? -90 : 90, &identify_ground_object_interrupt_callback));
    }
    else if (is_cliff) {
        if      (l_f)  cliff_type = l_f;
        else if (fl_f) cliff_type = fl_f;
        else if (fr_f) cliff_type = fr_f;
        else           cliff_type = r_f;

        ur_send_line("cliff");
        cliff_turn_direction = (r_f || fr_f) ? -90 : 90;
        if (fr_f)      cliff_turn_direction = -90;
        else if (fl_f) cliff_turn_direction = 90;

        cq_queue_front(gen_rotate_cmd_intr(cliff_turn_direction, &identify_cliff_interrupt_callback));

        if (fr_f)      cq_queue_front(gen_rotate_cmd(60)); //
        else if (fl_f) cq_queue_front(gen_rotate_cmd(-60)); //

    }

    return 1;
}
