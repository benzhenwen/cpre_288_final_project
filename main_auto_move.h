#pragma once


#include "main_scan_data.h"

#include "movement.h"
#include "command.h"
#include "movementcommands.h"

#include <stdint.h>



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



// cliff detection

#define BLACK_THRESH 300
#define WHITE_THRESH 2500

// returns 0: normal, 1: black, 2: white for any of the cliff sensors
// priority is left to right, however for this application the details are not neccesary.
char cliff_state(oi_t * sensor_data) {
    const uint16_t l_v  = (sensor_data->cliffLeftSignal);
    const uint16_t fl_v = (sensor_data->cliffFrontLeftSignal);
    const uint16_t fr_v = (sensor_data->cliffFrontRightSignal);
    const uint16_t r_v  = (sensor_data->cliffRightSignal);

    // flag value for each cliff - 0: normal, 1: black, 2: white
    const char l_f  = (l_v  < BLACK_THRESH) ? 1 : ((l_v  > WHITE_THRESH) ? 2 : 0);
    const char fl_f = (fl_v < BLACK_THRESH) ? 1 : ((fl_v > WHITE_THRESH) ? 2 : 0);
    const char fr_f = (fr_v < BLACK_THRESH) ? 1 : ((fr_v > WHITE_THRESH) ? 2 : 0);
    const char r_f  = (r_v  < BLACK_THRESH) ? 1 : ((r_v  > WHITE_THRESH) ? 2 : 0);

    if (l_f)  return l_f;
    if (fl_f) return fl_f;
    if (fr_f) return fr_f;
              return r_f;
}



// flags
float cliff_detect_angle_a;
float cliff_detect_angle_b;

int cliff_turn_direction;
char cliff_type;

// the second callback for the cliff identification
char identify_cliff_interrupt_callback_2(oi_t * sensor_data) {
    // keep turning if there is a cliff
    if (cliff_state(sensor_data)) return 0;

    // collect data point a
    move_stop();
    cliff_detect_angle_b = get_pos_r();

    // add the cliff to the map
    if (cliff_detect_angle_b < cliff_detect_angle_b) cliff_detect_angle_b += 360;
    float cliff_angle = ((cliff_detect_angle_a + cliff_detect_angle_b) / 2 + 180) * (M_PI / 180);

    const float cliff_object_rad = (cliff_type == 1) ? 100.0f : 262144.0f; // still maintaining about 0.025 float resolution while being large. DO RESEARCH BEFORE CHANGING THIS VALUE!!

    const float tx = cosf(cliff_angle) * (cliff_object_rad + 160);
    const float ty = sinf(cliff_angle) * (cliff_object_rad + 160);

    object_map[object_map_c] = (object_positional) { get_pos_x() + tx, get_pos_y() + ty, cliff_object_rad, (char) (cliff_type + 1) };
    object_map_c++;

    send_data_packet(object_map, object_map_c); // update python data packet

    return 1;
}

// the point of this callback is simply to turn until no cliff sensors are being triggered
char identify_cliff_interrupt_callback(oi_t * sensor_data) {
    // keep turning if there is a cliff
    if (cliff_state(sensor_data)) return 0;

    // collect data point a
    move_stop();
    cliff_detect_angle_a = get_pos_r();

    // rotate the opposite direction a little, then run the same routine to find when we are no longer triggering a cliff
    cq_queue(gen_rotate_cmd(180));
    cq_queue(gen_rotate_cmd_intr(-cliff_turn_direction, &identify_cliff_interrupt_callback_2));

    return 1;
}





char move_bump_interrupt_callback(oi_t * sensor_data) {

    // basic bump
    const char is_bump = sensor_data->bumpLeft || sensor_data->bumpRight;

    // basic cliff
    const uint16_t l_v  = (sensor_data->cliffLeftSignal);
//    const uint16_t fl_v = (sensor_data->cliffFrontLeftSignal);
    const uint16_t fr_v = (sensor_data->cliffFrontRightSignal);
    const uint16_t r_v  = (sensor_data->cliffRightSignal);

    // flag value for each cliff - 0: normal, 1: black, 2: white
    const char l_f  = (l_v  < BLACK_THRESH) ? 1 : ((l_v  > WHITE_THRESH) ? 2 : 0);
//    const char fl_f = (fl_v < BLACK_THRESH) ? 1 : ((fl_v > WHITE_THRESH) ? 2 : 0);
    const char fr_f = (fr_v < BLACK_THRESH) ? 1 : ((fr_v > WHITE_THRESH) ? 2 : 0);
    const char r_f  = (r_v  < BLACK_THRESH) ? 1 : ((r_v  > WHITE_THRESH) ? 2 : 0);

    const char is_cliff = cliff_state(sensor_data);

    // no interrupt check
    if (!is_bump && !is_cliff) return 0;

    move_stop();

    // bump handling
    if (is_bump) {
        ur_send_line("bump");
        cq_queue(gen_rotate_cmd_intr(sensor_data->bumpRight ? -90 : 90, &identify_ground_object_interrupt_callback));
    }
    else if (is_cliff) {
        ur_send_line("cliff");
        cliff_type = is_cliff;
        cliff_turn_direction = (r_f || fr_f) ? 120 : -120;
        if (!r_f && !l_f) cq_queue(gen_rotate_cmd((cliff_turn_direction > 0) ? 35 : -35));
        cq_queue(gen_rotate_cmd_intr(cliff_turn_direction, &identify_cliff_interrupt_callback));
    }

    return 1;
}
