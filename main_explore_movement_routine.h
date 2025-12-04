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
void update_weighted_map();




// queues the starting command - to enter the filed (and reset position data)
void explore_queue_start() {
    sound_startup();
    cq_queue(gen_move_cmd(TILE_SIZE_MM)); // move into the main tile
    cq_queue(gen_invoke_function_cmd(&explore_loop_scan)); // start the main loop
}

// the start of the auto loop. scans and then invokes explore_loop_path for path finding routine
void explore_loop_scan() {
    ur_send_line("explore loop scan start");

    // perform a scan and update the object map
    cq_queue(gen_invoke_function_cmd(&perform_scan_and_obj_detection));

    // start pathing and begin nav part
    cq_queue(gen_invoke_function_cmd(&explore_loop_path));
}

static float mx = 0, my = 0; // the point the bot has been instructed to move to
static float tx = 0, ty = 0; // the chosen target point (can be far away)
char attempt_persist_point = 0;

// tries to pathfind, pick a point to go to, then goes there
void explore_loop_path() {
    ur_send_line("path finding start");

    // the start point
    const float sx = get_pos_x();
    const float sy = get_pos_y();

    // pick a new random point to go to and try to path
    unsigned int attept_counter = 0;
    do {
        if (attept_counter++ > 256) {
            ur_send_line("path finding attempts >256 failed, auto aborting");
            move_stop();
            cq_clear();
            return;
        }

        if (!attempt_persist_point) { // let one attempt go by first to persist tx and ty
            // pick a random point to go to
            exp_map_pick_random_point(&tx, &ty);
        }

//        char buff[64];
//        sprintf(buff, "attempting path point: (%.0f, %.0f)", tx, ty);
//        ur_send_line(buff);

        // attempt to path to that point
        path_to(sx, sy, tx, ty, &mx, &my);

//        sprintf(buff, "attempting mid point: (%.0f, %.0f)", tx, ty);
//        ur_send_line(buff);

        // we just tried that point, if it was a turn only before we skipped generating a point, but we need to next cycle
        attempt_persist_point = 0;

        // attempt while the target point is not within 50mm (also invalid pathfinding returns 0 distance)
    } while (dist(sx, sy, mx, my) < 50);

    char buff[64];
    sprintf(buff, "go to: (%.0f, %.0f) mp: (%.0f, %.0f)", tx, ty, mx, my);
    ur_send_line(buff);


    // get the angle bearing to that point
    const float target_angle_bearing = calculate_relative_target_r(atan2f(my - sy, mx - sx) * (180 / M_PI));

    // exclusively turn if we are rotating more than 55 degrees
    if (abs(target_angle_bearing - get_pos_r()) > 55) {
        ur_send_line("rotating to face point");

        cq_queue(gen_rotate_to_cmd(target_angle_bearing));
    }
    // else we move some distance in that direction
    else {
        // the distance we travel is the min of the distance of our target point, or some small distance
        const float dest_dist = dist(sx, sy, mx, my);
        const float move_dist = MIN(300.0f, dest_dist);

        if (dest_dist == 0) {
            ur_send_line("dest dist 0 error");
        }

        // calculate the real destination point
        const float dex = lerp(sx, mx, move_dist/dest_dist);
        const float dey = lerp(sy, my, move_dist/dest_dist);

        sprintf(buff, "driving to: (%.0f, %.0f)", dex, dey);
        ur_send_line(buff);

        // try to go there!
        cq_queue(gen_move_to_cmd_intr(dex, dey, &move_bump_interrupt_callback));

        // update the weighted map after a successful movement
        cq_queue(gen_invoke_function_cmd(&update_weighted_map));
    }

    attempt_persist_point = 1;

    // restart the loop!
    cq_queue(gen_invoke_function_cmd(&explore_loop_scan));

    ur_send_line("pathing function done");

}

// void parameter function wrapper for exp_map_new_searched_point(posx, posy)
void update_weighted_map() {
    exp_map_new_searched_point(get_pos_x(), get_pos_y());
}
