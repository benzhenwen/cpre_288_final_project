#include <stdlib.h>
#include <math.h>

#include "open_interface.h"
#include "movement.h"

// ---------- config stuff -----------

// old slow values: 150 80  110 30

const int _movement_speed = 180; // 0 - 500
const int _rotation_speed = 120;

const int _min_movement_speed = 120; // 0 - 500
const int _min_rotation_speed = 25;

float _movement_scaler = 1; // TODO - made modifiable
float _rotation_scaler = 1.007;



// ---------- position / rotation data ------------
inline float lerp(float a, float b, float f) {
    return a * (1.0 - f) + (b * f);
}
inline float dist2(float ax, float ay, float bx, float by) {
    float dx = bx - ax;
    float dy = by - ay;
    return dx * dx + dy * dy;
}
inline float dist(float ax, float ay, float bx, float by) {
    return sqrtf(dist2(ax, ay, bx, by));
}

float pos_x = 0;
float pos_y = 0;
float pos_r = 0;

float get_pos_x() { return pos_x; }
float get_pos_y() { return pos_y; }
float get_pos_r() { return pos_r; }

float target_x = 0;
float target_y = 0;
float target_r = 0;

float get_target_x() { return target_x; }
float get_target_y() { return target_y; }
float get_target_r() { return target_r; }

void reset_pos() {
    pos_x = 0;
    pos_y = 0;
    pos_r = 0;
    target_x = 0;
    target_y = 0;
    target_r = 0;
    move_stop();
}

float calculate_relative_target_r(float r) {
    while (r < 0) r += 360; // get positive
    r = fmod(r, 360); // bound 0-360

    if ((pos_r - r) > 180) return r + 360;
    else if ((r - pos_r) > 180) return r - 360;
    else return r;
}

char active_movement_flag = 0;
char done_moving_flag = 1;

char rotating_to_face_target_flag = 0;
char move_mode_flag = 0; // 0 = move, 1 = rotate
char get_move_mode_flag() { return move_mode_flag; }

char move_reverse_flag = 0;

float apprach_distance_offset = 0; // positive values only. setting this higher will have the bot try to approach the object this many mm from the bot
float get_target_apprach_distance_offset() { return apprach_distance_offset; }

void update_position_data(oi_t * sensor_data) {
    pos_x += cosf(pos_r * (M_PI / 180)) * sensor_data->distance * _movement_scaler;
    pos_y += sinf(pos_r * (M_PI / 180)) * sensor_data->distance * _movement_scaler;
    pos_r += sensor_data->angle * _rotation_scaler;

    // angle bounds, always update target_r with pos_r
    if (pos_r >= 360) {
        pos_r -= 360;
        target_r -= 360;
    }
    else if (pos_r < 0) {
        pos_r += 360;
        target_r += 360;
    }

    if (active_movement_flag == 0) {
        done_moving_flag = 1;
        return;
    }

    // movement
    done_moving_flag = 0;

    float dist_from_target = dist(pos_x, pos_y, target_x, target_y) - apprach_distance_offset;
    if (dist_from_target > 5 && move_mode_flag == 0) {
        float target_r_m = atan2(target_y - pos_y, target_x - pos_x) * 180 / M_PI; // rad to deg, to target
        if (move_reverse_flag) target_r_m += 180;
        target_r_m = calculate_relative_target_r(target_r_m);

        if (abs(target_r_m - pos_r) > 4) rotating_to_face_target_flag = 1; // over 4 degrees off, aim to rotate to target
        if (rotating_to_face_target_flag) {
            if (abs(target_r_m - pos_r) > 0.5) { // rotate to target, with higher precision
                float move_speed_rot = abs(lerp(_min_rotation_speed, _rotation_speed, MIN(1, abs(target_r_m - pos_r) / 20)));

                if ((target_r_m - pos_r) > 0) oi_setWheels(move_speed_rot, -move_speed_rot); // rotate cc
                else                          oi_setWheels(-move_speed_rot, move_speed_rot); // rotate c
            }
            else rotating_to_face_target_flag = 0; // once we've reached that higher precision, don't attempt a dedicated rotate to target until we're off by a substantial amount
        }

        else { // move to that spot
            float target_r_v = target_r_m;
            const int virt_point_dist = 80;

            if (dist_from_target > virt_point_dist) {
                float virtual_x = lerp(pos_x, target_x, virt_point_dist / dist_from_target); // define a point 100 units away from the robot, in the direction of our target
                float virtual_y = lerp(pos_y, target_y, virt_point_dist / dist_from_target);
                target_r_v = atan2(virtual_y - pos_y, virtual_x - pos_x) * 180 / M_PI; // rad to deg, to virtual target
                if (move_reverse_flag) target_r_v += 180;
                target_r_v = calculate_relative_target_r(target_r_v);
            }

            const float move_offset = move_reverse_flag ? -50 : 50;

            float move_speed_line = lerp(_min_movement_speed, _movement_speed, MIN(1, dist_from_target / 150));
            float move_speed_l_w = lerp(move_speed_line - move_offset, move_speed_line, /**/ MIN(1, -MIN(1, (target_r_v - pos_r) / 2) + 1) /* slow down the left motor when angle offset is positive (rotate cc) */ );
            float move_speed_r_w = lerp(move_speed_line - move_offset, move_speed_line, /**/ MIN(1, -MIN(1, (pos_r - target_r_v) / 2) + 1) /* slow down the right motor when angle offset is positive (rotate c) */ );

            if (!move_reverse_flag) oi_setWheels(move_speed_r_w, move_speed_l_w); // move forward
            else                    oi_setWheels(-move_speed_r_w * 0.75, -move_speed_l_w * 0.75); // move backward
        }
    }
    else if (abs(target_r - pos_r) > 0.5 && move_mode_flag == 1) { // aim to rotate in target direction
        float move_speed_rot = lerp(_min_rotation_speed, _rotation_speed, MIN(1, abs(target_r - pos_r) / 20));

        if ((target_r - pos_r) > 0) oi_setWheels(move_speed_rot, -move_speed_rot); // rotate cc
        else                        oi_setWheels(-move_speed_rot, move_speed_rot); // rotate c
    }
    else {
        active_movement_flag = 0;
        done_moving_flag = 1;
    }
}






// ---------- movement commands --------------


void start_linear_move(CommandData * data) {
    active_movement_flag = 1;

    move_mode_flag = 0;
    move_reverse_flag = 0;
    apprach_distance_offset = 0;
    target_x += cosf(target_r * (M_PI / 180)) * data->move.distance;
    target_y += sinf(target_r * (M_PI / 180)) * data->move.distance;
}
void start_reverse_move(CommandData * data) {
    active_movement_flag = 1;

    move_mode_flag = 0;
    move_reverse_flag = 1;
    apprach_distance_offset = 0;
    target_x -= cosf(target_r * (M_PI / 180)) * data->move.distance;
    target_y -= sinf(target_r * (M_PI / 180)) * data->move.distance;
}
void start_approach_move(CommandData * data) {
    active_movement_flag = 1;

    move_mode_flag = 0;
    move_reverse_flag = 0;
    apprach_distance_offset = data->moveTo.apprach_rad;
    target_x = data->moveTo.x;
    target_y = data->moveTo.y;
}

void start_rotate_move(CommandData * data) {
    active_movement_flag = 1;

    move_mode_flag = 1;
    move_reverse_flag = 0;
    target_r = calculate_relative_target_r(target_r + data->move.distance);
}
void start_rotate_move_to(CommandData * data) {
    active_movement_flag = 1;

    move_mode_flag = 1;
    move_reverse_flag = 0;
    target_r = calculate_relative_target_r(data->move.distance);
}

void move_stop() {
    active_movement_flag = 0;
    done_moving_flag = 1;

    oi_setWheels(0, 0);
    target_x = pos_x;
    target_y = pos_y;
    target_r = pos_r;
}

char move_end_cond(oi_t * sensor_data) {
    if (done_moving_flag) oi_setWheels(0, 0);
    return done_moving_flag;
}
