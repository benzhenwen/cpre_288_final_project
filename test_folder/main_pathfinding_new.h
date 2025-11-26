#include "main_scan_data.h"
#include "rng.h"

#include <math.h>
#include <stdint.h>







// ------------------------------ exploratory heat map ------------------------------
// this is a 16 by 16 grid mapped to a 5m by 5m square, starting at (0, 0). each contains a 4 bit value
// each point is 333.333333mm spaced in a grid, starting at center. if a new found point goes outside the grid (it only ever should in the neg direction), it will adjust itself
// this allows for travel in one direction of about 5m
// the y direction is represented by the array, where positive is up
// the x direction is represented by each 64 bit number, divided into 16 chunks of 4 bits each
// only 128 bytes!

#define EXP_MAP_SPACING 333.3333333f // space between each point

static uint64_t exploratory_map[16];
int8_t y_offset = 0;
int8_t x_offset = 0;

#define exp_map_val_at(x, y) ((unsigned int) ((exploratory_map[y] >> (4 * x)) & 0b1111))

static inline void exp_map_set_at(unsigned int x, unsigned int y, unsigned int value) {
    exploratory_map[y] = (exploratory_map[y] & ~(((uint64_t) 0b1111) << (4 * x))) | (((uint64_t) (value & 0b1111)) << (4 * x));
}

static void exp_map_shift_left() {
    x_offset--;
    int y;
    for (y = 0; y < 16; y++) {
        exploratory_map[y] = exploratory_map[y] >> 4;
    }
}
static void exp_map_shift_down() {
    y_offset--;
    int y;
    for (y = 0; y < 15; y++) {
        exploratory_map[y] = exploratory_map[y+1];
    }
    exploratory_map[15] = 0;
}
static void exp_map_dec_all() {
    int x, y;
    for (x = 0; x < 16; x++) {
        for (y = 0; y < 16; y++) {
            int map_val = exp_map_val_at(x, y);
            if (map_val > 0) map_val--;
            exp_map_set_at(x, y, map_val);
        }
    }
}

static inline void exp_map_new_searched_point(float sx, float sy) {
    // shift the map as needed
    while (sx < x_offset * EXP_MAP_SPACING) {
        exp_map_shift_left();
    }
    while (sy < y_offset * EXP_MAP_SPACING) {
        exp_map_shift_down();
    }

    // from global to map
    const int gx = roundf((sx - (x_offset * EXP_MAP_SPACING)) / EXP_MAP_SPACING);
    const int gy = roundf((sy - (y_offset * EXP_MAP_SPACING)) / EXP_MAP_SPACING);

    if (gx < 0 || gx > 15 || gy < 0 || gy > 15) ur_send_line("exp_map_new_searched_point out of bounds error");

    int map_val = exp_map_val_at(gx, gy);
    if (map_val >= 0b1111) {
        exp_map_dec_all();
        map_val--;
    }
    exp_map_set_at(gx, gy, map_val + 1);
}
static inline unsigned int exp_map_get_weighted_point(float sx, float sy) {
    const int gx = roundf((sx - (x_offset * EXP_MAP_SPACING)) / EXP_MAP_SPACING);
    const int gy = roundf((sy - (y_offset * EXP_MAP_SPACING)) / EXP_MAP_SPACING); 

    if (gx < 0 || gx > 15 || gy < 0 || gy > 15) ur_send_line("exp_map_get_weighted_point out of bounds error");

    return exp_map_val_at(gx, gy);
}
static inline unsigned int exp_map_get_weighted_point_safe(float sx, float sy) {
    const int gx = roundf((sx - (x_offset * EXP_MAP_SPACING)) / EXP_MAP_SPACING);
    const int gy = roundf((sy - (y_offset * EXP_MAP_SPACING)) / EXP_MAP_SPACING); 

    if (gx < 0 || gx > 15 || gy < 0 || gy > 15) {
        return roundf(exp_rand_range(0, 3)); // generally bias to outside the map, this will help us find walls faster
    }

    return exp_map_val_at(gx, gy);
}



// ------------------------------ pathfinding helpers ------------------------------
#define BOT_RADIUS 160
#define CLEARANCE_TOLERANCE 10

// returns 1 if point (px,py) is in free space (not colliding with any inflated object)
static char is_point_free(float px, float py) {
    int i;
    for (i = 0; i < object_map_c; ++i) {
        const object_positional const *o = &object_map[i];

        float r = o->radius + BOT_RADIUS + CLEARANCE_TOLERANCE;
        float dx = px - o->x;
        float dy = py - o->y;
        float d2 = dx * dx + dy * dy;

        if (d2 <= r * r) {
            // Inside or touching inflated obstacle
            return 0;
        }
    }
    return 1;
}

// returns 1 if segment AB is collision-free against all inflated objects
static char segment_clear(float ax, float ay, float bx, float by) {
    int i;
    float dx = bx - ax;
    float dy = by - ay;
    float seg_len2 = dx * dx + dy * dy;

    if (seg_len2 == 0.0f) {
        // Degenerate segment, just test the point.
        return is_point_free(ax, ay);
    }

    for (i = 0; i < object_map_c; ++i) {
        const object_positional const *o = &object_map[i];

        float r = o->radius + BOT_RADIUS + CLEARANCE_TOLERANCE;

        // Vector from circle center to segment start
        float fx = ax - o->x;
        float fy = ay - o->y;

        // Project center onto segment (parameter t in [0,1])
        float t = -(fx * dx + fy * dy) / seg_len2;
        if (t < 0.0f) t = 0.0f;
        else if (t > 1.0f) t = 1.0f;

        float closest_x = ax + t * dx;
        float closest_y = ay + t * dy;

        float cx = closest_x - o->x;
        float cy = closest_y - o->y;
        float dist2 = cx * cx + cy * cy;

        if (dist2 <= r * r) {
            // The path comes within clearance of this obstacle
            return 0;
        }
    }

    return 1;
}


// ------------------------------ random point selection ------------------------------
#define GLOBAL_HALF_SIZE_MM 5000.0f
#define GLOBAL_MIN_MM (-GLOBAL_HALF_SIZE_MM)
#define GLOBAL_MAX_MM ( GLOBAL_HALF_SIZE_MM)

typedef struct SamplePoint {
    int16_t x, y;
} SamplePoint;

#define SAMPLE_POINT_CNT 64
static void exp_map_pick_random_point(float * ox, float * oy) {
    // generate a set of random points to try, should all be not inside of obstacle
    SamplePoint sample_points[SAMPLE_POINT_CNT];
    int i;
    for (i = 0; i < SAMPLE_POINT_CNT; i++) {
        do {
            sample_points[i].x = roundf(exp_rand_range(-5000, 5000));
            sample_points[i].y = roundf(exp_rand_range(-5000, 5000));
        } while (!is_point_free(sample_points[i].x, sample_points[i].y));
    }


    // find the lowest sample point
    unsigned int lowest_index = 0;
    unsigned int lowest_value = exp_map_get_weighted_point_safe(sample_points[0].x, sample_points[0].y);

    for (i = 1; i < SAMPLE_POINT_CNT; i++) {
        unsigned int value = exp_map_get_weighted_point_safe(sample_points[i].x, sample_points[i].y);

        if (value < lowest_value) {
            lowest_index = i;
            lowest_value = value;
        }
    }

    *ox = sample_points[lowest_index].x;
    *oy = sample_points[lowest_index].y;
}


