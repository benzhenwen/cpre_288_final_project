#pragma once

#include <stdint.h>

extern float get_pos_x();
extern float get_pos_y();

static uint32_t exp_rng_state = 1u;

// RNG system, uses position of the bot to introduce physical randomness
static float exp_rand() {
    // Simple LCG mixed with some pose-based noise
    uint32_t px = (uint32_t) ((get_pos_x() + 2) * 100000.0f);
    uint32_t py = (uint32_t) ((get_pos_y() + 2) * 100000.0f);

    exp_rng_state = exp_rng_state * 1664525u + 1013904223u;
    exp_rng_state ^= (px + (py << 16));

    // Use low 24 bits as a fraction in [0,1)
    return (float)(exp_rng_state & 0x00FFFFFFu) / 16777216.0f;
}


// gen random number in range using exp_rand
static float exp_rand_range(float min_v, float max_v) {
    return min_v + exp_rand() * (max_v - min_v);
}
