
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

// increase the nearest weight on the map by 1 given the robot's position
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

// get a weighted point on the map from bot position
static inline unsigned int exp_map_get_weighted_point(float sx, float sy) {
    const int gx = roundf((sx - (x_offset * EXP_MAP_SPACING)) / EXP_MAP_SPACING);
    const int gy = roundf((sy - (y_offset * EXP_MAP_SPACING)) / EXP_MAP_SPACING);

    if (gx < 0 || gx > 15 || gy < 0 || gy > 15) ur_send_line("exp_map_get_weighted_point out of bounds error");

    return exp_map_val_at(gx, gy);
}

// get a weighted point on the map from bot position, ensures safe out-of-bounds behavior
static inline unsigned int exp_map_get_weighted_point_safe(float sx, float sy) {
    const int gx = roundf((sx - (x_offset * EXP_MAP_SPACING)) / EXP_MAP_SPACING);
    const int gy = roundf((sy - (y_offset * EXP_MAP_SPACING)) / EXP_MAP_SPACING);

    if (gx < 0 || gx > 15 || gy < 0 || gy > 15) {
        return roundf(exp_rand_range(0, 3)); // generally bias to outside the map, this will help us find walls faster
    }

    return exp_map_val_at(gx, gy);
}



// ------------------------------ pathfinding helpers ------------------------------
// the radius of the bot and the space between objects we want to maintain, accounting for error in some cases
#define BOT_RADIUS 160
#define CLEARANCE_TOLERANCE 30

// returns if the object at index is brushed up against an object, defined as dist < BOT_RADIUS + CLEARANCE_TOLERANCE
static inline char is_object_brushing_bot(int index) {
    const object_positional *o = &object_map[index];
    const float distance = dist(o->x, o->y, get_pos_x(), get_pos_y());
    return distance < BOT_RADIUS + CLEARANCE_TOLERANCE;
}

// returns 1 if point (px,py) is in free space (not colliding with any inflated object)
static char is_point_free(float px, float py) {
    int i;
    for (i = 0; i < object_map_c; ++i) {
        const object_positional *o = &object_map[i];

        // duplicated on segment_clear, we do not consider the tolerance when too nearby an object for the sake of pathfinding
        float r = o->radius + BOT_RADIUS + (is_object_brushing_bot(i) ? 0 : CLEARANCE_TOLERANCE);

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
        const object_positional *o = &object_map[i];

        float r = o->radius + BOT_RADIUS + (is_object_brushing_bot(i) ? 0 : CLEARANCE_TOLERANCE);

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

// selects a random valid xy point -5000 to 5000, prioritizes low values in the exp_map (ie, lesser explored points)
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





// ------------------------------ main pathfinding algorithm ------------------------------
#define MAX_CANDIDATES 64
#define DUPLICATE_EPS2 (25.0f)   // 5 mm squared



// Add a candidate waypoint if it's free and not (almost) duplicate.
static void add_candidate(float x, float y, float *cand_x, float *cand_y, int *cand_count) {
    int i;

    if (*cand_count >= MAX_CANDIDATES) {
        ur_send_line("MAX_CANDIDATES overflow in add_candidate");
        return;
    }

    if (!is_point_free(x, y)) {
        return;
    }

    for (i = 0; i < *cand_count; ++i) {
        float dx = x - cand_x[i];
        float dy = y - cand_y[i];
        if (dx * dx + dy * dy < DUPLICATE_EPS2) {
            return;
        }
    }

    cand_x[*cand_count] = x;
    cand_y[*cand_count] = y;
    (*cand_count)++;
}

// S - W1 - W2 - E path finding algorithm based off of candidate points around objects
// ox, oy are return values
// returns sx, sy if no valid path found
// returns tx, ty if the valid path is completely clear
// returns W1 (the first waypoint to go to in the path) otherwise
void path_to(float sx, float sy, float tx, float ty, float *ox, float *oy) {
    int i;
    float cand_x[MAX_CANDIDATES];
    float cand_y[MAX_CANDIDATES];
    int   cand_count = 0;

    const float BIG = 1.0e30f;

    // 0. Safety: start/target must be in free space, otherwise fail.
    if (!is_point_free(sx, sy) || !is_point_free(tx, ty)) {
        *ox = sx;
        *oy = sy;
        return;
    }

    // 1. If direct segment is clear, go straight to target.
    if (segment_clear(sx, sy, tx, ty)) {
        *ox = tx;
        *oy = ty;
        return;
    }

    // 2. Build diversion waypoints around obstacles that block S->T.
    {
        float dx = tx - sx;
        float dy = ty - sy;
        float L2 = dx * dx + dy * dy;

        if (L2 == 0.0f) {
            *ox = sx;
            *oy = sy;
            return;
        }

        float L  = sqrtf(L2);
        float ux = dx / L;
        float uy = dy / L;
        // Perpendicular to path (unit)
        float nx = -uy;
        float ny =  ux;

        // Cluster info for blocking obstacles (for "walls")
        int   blocker_count   = 0;
        float min_t           = 0.0f;
        float max_t           = 0.0f;
        float max_abs_perp    = 0.0f;
        float max_r_inflated  = 0.0f;

        for (i = 0; i < object_map_c; ++i) {
            object_positional *o = &object_map[i];
            float r_inflated = o->radius + BOT_RADIUS + CLEARANCE_TOLERANCE;

            // Vector from S to obstacle center
            float wx = o->x - sx;
            float wy = o->y - sy;

            // Longitudinal coordinate along S->T
            float t = wx * ux + wy * uy;

            // Ignore obstacles far behind start or far beyond target
            if (t < -r_inflated || t > L + r_inflated)
                continue;

            // Signed perpendicular distance to S->T
            float d_perp_signed = wx * nx + wy * ny;
            float d2_perp       = d_perp_signed * d_perp_signed;

            // If outside inflated corridor, doesn't block
            if (d2_perp >= r_inflated * r_inflated)
                continue;

            // === This obstacle blocks the corridor ===
            if (blocker_count == 0) {
                min_t = max_t = t;
            } else {
                if (t < min_t) min_t = t;
                if (t > max_t) max_t = t;
            }
            blocker_count++;
            if (fabsf(d_perp_signed) > max_abs_perp)
                max_abs_perp = fabsf(d_perp_signed);
            if (r_inflated > max_r_inflated)
                max_r_inflated = r_inflated;

            // --- Local waypoints around this obstacle ---

            // Side offset (perpendicular)
            const float SIDE_OFFSET_SCALE_LOCAL = 1.05f;   // hug closer
            float side_offset = r_inflated * SIDE_OFFSET_SCALE_LOCAL;
            float min_side_offset = r_inflated + 2.0f;     // small safety
            if (side_offset < min_side_offset) side_offset = min_side_offset;

            // Forward/back offset along the path for 2-node wrapping
            const float FORWARD_SCALE = 0.7f;
            float f_offset = r_inflated * FORWARD_SCALE;

            int sgn;
            for (sgn = -1; sgn <= 1; sgn += 2) {
                // Base side point (left/right of obstacle)
                float base_x = o->x + nx * (sgn * side_offset);
                float base_y = o->y + ny * (sgn * side_offset);

                // Pure side-step candidate
                add_candidate(base_x, base_y, cand_x, cand_y, &cand_count);

                // Backward along path
                add_candidate(base_x - ux * f_offset,
                              base_y - uy * f_offset,
                              cand_x, cand_y, &cand_count);

                // Forward along path
                add_candidate(base_x + ux * f_offset,
                              base_y + uy * f_offset,
                              cand_x, cand_y, &cand_count);
            }
        }

        // Cluster-level "far" waypoints to go around a whole wall,
        // but only if there is more than one blocking obstacle.
        if (blocker_count > 1 && max_r_inflated > 0.0f) {
            float t_mid = 0.5f * (min_t + max_t);
            if (t_mid < 0.0f) t_mid = 0.0f;
            if (t_mid > L)    t_mid = L;

            float base_x = sx + ux * t_mid;
            float base_y = sy + uy * t_mid;

            const float SIDE_OFFSET_SCALE_CLUSTER = 1.8f; // smaller than before
            float offset = max_abs_perp +
                           SIDE_OFFSET_SCALE_CLUSTER * max_r_inflated +
                           CLEARANCE_TOLERANCE;

            if (offset < max_r_inflated + CLEARANCE_TOLERANCE)
                offset = max_r_inflated + CLEARANCE_TOLERANCE;

            // Above / one side
            add_candidate(base_x + nx * offset,
                          base_y + ny * offset,
                          cand_x, cand_y, &cand_count);

            // Below / other side
            add_candidate(base_x - nx * offset,
                          base_y - ny * offset,
                          cand_x, cand_y, &cand_count);
        }
    }

    if (cand_count == 0) {
        // No diversion points found, fail.
        *ox = sx;
        *oy = sy;
        return;
    }

    // 3. Try all 2-node paths: S -> W1 -> W2 -> T (preferred).
    {
        float best_cost = BIG;
        float best_x = sx;
        float best_y = sy;
        int i1, i2;

        for (i1 = 0; i1 < cand_count; ++i1) {
            float w1x = cand_x[i1];
            float w1y = cand_y[i1];

            if (!segment_clear(sx, sy, w1x, w1y)) continue;

            for (i2 = 0; i2 < cand_count; ++i2) {
                float w2x, w2y;
                float cost;

                if (i2 == i1) continue;

                w2x = cand_x[i2];
                w2y = cand_y[i2];

                if (!segment_clear(w1x, w1y, w2x, w2y)) continue;
                if (!segment_clear(w2x, w2y, tx, ty)) continue;

                cost  = dist(sx, sy, w1x, w1y);
                cost += dist(w1x, w1y, w2x, w2y);
                cost += dist(w2x, w2y, tx, ty);

                if (cost < best_cost) {
                    best_cost = cost;
                    best_x = w1x; // next node is the first waypoint
                    best_y = w1y;
                }
            }
        }

        if (best_cost < BIG * 0.5f) {
            *ox = best_x;
            *oy = best_y;
            return;
        }
    }

    // 4. Fallback: 1-node paths S -> W -> T if 2-node failed.
    {
        float best_cost = BIG;
        float best_x = sx;
        float best_y = sy;
        int   k;

        for (k = 0; k < cand_count; ++k) {
            float wx = cand_x[k];
            float wy = cand_y[k];

            if (!segment_clear(sx, sy, wx, wy)) continue;
            if (!segment_clear(wx, wy, tx, ty)) continue;

            {
                float cost = dist(sx, sy, wx, wy) + dist(wx, wy, tx, ty);
                if (cost < best_cost) {
                    best_cost = cost;
                    best_x = wx;
                    best_y = wy;
                }
            }
        }

        if (best_cost < BIG * 0.5f) {
            *ox = best_x;
            *oy = best_y;
            return;
        }
    }

    // 5. No path found with up to 2 waypoints -> fail (stay put).
    *ox = sx;
    *oy = sy;
}
