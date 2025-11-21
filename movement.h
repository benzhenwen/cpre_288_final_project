#pragma once

#include "command.h"

// ---------- position / rotation data ------------

// gets position data, self-explanatory
float get_pos_x();
float get_pos_y();
float get_pos_r();

void reset_pos();

float get_target_x();
float get_target_y();
float get_target_r();

char get_move_mode_flag();
float get_target_apprach_distance_offset();

void update_position_data(oi_t * sensor_data);

// ---------- util ------------

// init - call at start
void movement_init();

// free - call at end
void movement_free();


// ---------- config stuff -----------
void set_movement_speed(int speed); // sets the movement speed 0-500 units

// ---------- movement commands --------------

void start_linear_move(CommandData * data); // start moving straight, uses MoveCD
void start_reverse_move(CommandData * data); // start reversing straight, uses MoveCD
void start_approach_move(CommandData * data); // move to a point, uses MoveToCD
void start_rotate_move(CommandData * data); // start rotating, uses MoveCD
void start_rotate_move_to(CommandData * data); // start rotating, uses MoveCD
void move_stop(); // stop moving
char move_end_cond(oi_t * sensor_data); // the end condition for the command





// returns a command that will make the robot go forward some distance, works with negative values
inline Command gen_move_cmd(float distance);
