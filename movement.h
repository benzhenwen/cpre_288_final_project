#pragma once

#include "command.h"

// ---------- position / rotation data ------------

// gets position data, self-explanatory
float get_pos_x();
float get_pos_y();
float get_pos_r();

// reset position and target to 0, 0, 0
void reset_pos();

// get the bot's target position
float get_target_x();
float get_target_y();
float get_target_r();

// access the move mode of the bot, 0 is move and 1 is rotate
char get_move_mode_flag();
// the approach distance
float get_target_apprach_distance_offset();

// the main loop call. updates the encoded position of the robot based off of sensor data and tries to move to target pos if not already there
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


// external export util
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

float calculate_relative_target_r(float r);
inline float lerp(float a, float b, float f);
inline float dist2(float ax, float ay, float bx, float by);
inline float dist(float ax, float ay, float bx, float by);
