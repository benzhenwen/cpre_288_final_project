#pragma once

#include "movement.h"
#include "command.h"
#include "untilcommands.h"

#include "main_scan_data.h"
#include "main_objects.h"
#include "main_auto_move.h"

// the main exploratory routine for the bot. its goal is to seek out the objective area


// some useful constants
#define TILE_SIZE_MM 610

#ifndef BOT_RADIUS
    #define BOT_RADIUS 160
#endif


// function defs
void explore_queue_start();
void explore_loop_scan();
void explore_loop_path();




// queues the starting command - to enter the filed (and reset position data)
void explore_queue_start() {
    cq_queue(gen_move_cmd(TILE_SIZE_MM)); // move into the main tile
    cq_queue(gen_invoke_function_cmd(&explore_loop_scan)); // start the main loop
}

void explore_loop_scan() {
    // perform a scan and update the object map
    cq_queue(gen_invoke_function_cmd(&perform_scan_and_obj_detection));

    // start pathing and begin nav part
    cq_queue(gen_invoke_function_cmd(&explore_loop_path));
}

void explore_loop_path() {
    // pick a random point to go to
    float tx, ty; // the chosen target point
    exp_map_pick_random_point(&tx, &ty);

    // attempt to path to that point
    float mx, my; // the point the bot has been instructed to move to
    path_to(get_target_x(), get_target_y(), tx, ty, &mx, &my);

    // move just a little to that point, or turn to face the correct direction.
    // if move >0.5m or rotate >50deg, only move .5m or turn respectively, then request a new scan
    // TODO
}
