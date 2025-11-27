#pragma once

#include <math.h>
#include <stdio.h>

// distance between two points
float distance_between(float x1, float y1, float x2, float y2) {
    return sqrtf(powf(x1-x2, 2) + powf(y1-y2, 2));
}

// produces the distance of a point from a line where
// x1 y1 is the ray's first point
// x2 y2 is the ray's second point
// x3 y3 is the point to check distance from the ray for
float distance_to_line(float x1, float y1, float x2, float y2, float x3, float y3) {
    float m = (y2 - y1) / (x2 - x1);
    float intercept_x = (y3 - y1 + x1*m + x3/m) / (m + 1/m);
    float intercept_y = m*(intercept_x - x1) + y1;

    if (intercept_y > fmaxf(y1, y2)) {
        if (y1 > y2) return distance_between(x1, y1, x3, y3);
                     return distance_between(x2, y2, x3, y3);
    }
    if (intercept_y < fminf(y1, y2)) {
        if (y1 < y2) return distance_between(x1, y1, x3, y3);
                     return distance_between(x2, y2, x3, y3);
    }

    return distance_between(intercept_x, intercept_y, x3, y3);
}

#define CLEARANCE_TOLERANCE 10
char point_intercepts_obj(float x, float y) {
    int i;
    for (i = 0; i < object_map_c; i++) {
        float dist_to_obj = distance_between(x, y, object_map[i].x, object_map[i].y);
        if (dist_to_obj < 160 + CLEARANCE_TOLERANCE + object_map[i].radius) return 1;
    } return 0;
}

#define SCAN_STEP_DIST 20.0f
char test_valid_path(float x1, float y1, float x2, float y2, float end_tolerance) {

    // trunc end for tolerance (like if we are trying to move directly to an object)
    if (end_tolerance > 0) {
        end_tolerance += 160 + CLEARANCE_TOLERANCE;

        float s = atan2f(y2-y1, x2-x1);
        x2 -= cosf(s) * end_tolerance;
        y2 -= sinf(s) * end_tolerance;
    }

    int i;
    for (i = 0; i < object_map_c; i++) {
        float dist_to_obj = distance_to_line(x1, y1, x2, y2, object_map[i].x, object_map[i].y);
        if (dist_to_obj < 160 + CLEARANCE_TOLERANCE + object_map[i].radius) return 0;
    }

    return 1;
}


// tries to find an approach angle for a target location from a start location
// seaches points around the object, then validates 2 direct paths from the start, the approach point, and the end point
// sx sy - the start location to searh
// tx ty - the target location
// min_dist - the minimum distance from the target to stand from
// max_dist - the maximum distance from the target to stand from
// end_tolerance - if approaching an object, pass in the object radius
// ox oy - the output x and y, returns as tx and ty if failed to find

#define ANGLE_STEP 5
#define APPROACH_STEP 25
void find_valid_approach(float sx, float sy, float tx, float ty, float min_dist, float max_dist, float end_tolerance, float * ox, float * oy) {
    float mx; // the midpoint to check
    float my;

    float approach_angle = atan2f(sy-ty, sx-tx);

    float approach_angle_offset;
    float approach_dist;
    for (approach_angle_offset = 0; approach_angle_offset <= 180; approach_angle_offset = (approach_angle_offset <= 0) ? -approach_angle_offset + ANGLE_STEP : -approach_angle_offset) {
        for (approach_dist = min_dist; approach_dist < max_dist; approach_dist += APPROACH_STEP) {

            // find the approach
            float a = approach_angle + (approach_angle_offset * M_PI / 180);

            mx = tx + cosf(a) * approach_dist;
            my = ty + sinf(a) * approach_dist;

            if (test_valid_path(sx, sy, mx, my, 0) && test_valid_path(mx, my, tx, ty, end_tolerance)) {
                char buff[32];
                sprintf(buff, "path mid: (%.2f, %.2f)", mx, my);
                ur_send_line(buff);

                *ox = mx;
                *oy = my;
                return;
            }
        }
    }

    ur_send_line("failed to find path");

    *ox = tx;
    *ox = ty;
    return;
}
