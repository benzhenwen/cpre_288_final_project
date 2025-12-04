#pragma once

#include "command.h"

#include "movement.h"

// -------------------------------- a header file containing function definitions for creating movement commands -------------------------------------
/*
 * each gen has a _intr variant, allowing you to set the interrupt condition
 * default behavior for interrupts is always_false
 * see command.h for more details
 */


// move bot forward
inline Command gen_move_cmd_intr(float distance, char (*interrupt_condition)(oi_t * sensor_data)) {
    CommandData cd;
    cd.move = (MoveCD) {distance};

    Command c;
    c.on_start = &start_linear_move;
    c.is_complete = &move_end_cond;
    c.is_interrupt = interrupt_condition;
    c.data = cd;

    return c;
}
inline Command gen_move_cmd(float distance) {
    return gen_move_cmd_intr(distance, &always_false);
}

// gen reverse command
inline Command gen_move_reverse_cmd_intr(float distance, char (*interrupt_condition)(oi_t * sensor_data)) {
    CommandData cd;
    cd.move = (MoveCD) {distance};

    Command c;
    c.on_start = &start_reverse_move;
    c.is_complete = &move_end_cond;
    c.is_interrupt = interrupt_condition;
    c.data = cd;

    return c;
}
inline Command gen_move_reverse_cmd(float distance) {
    return gen_move_reverse_cmd_intr(distance, &always_false);
}

// gen approach
inline Command gen_approach_cmd_intr(float x, float y, float rad, char (*interrupt_condition)(oi_t * sensor_data)) {
    CommandData cd;
    cd.moveTo = (MoveToCD) {x, y, rad};

    Command c;
    c.on_start = &start_approach_move;
    c.is_complete = &move_end_cond;
    c.is_interrupt = interrupt_condition;
    c.data = cd;

    return c;
}
inline Command gen_appraoch_cmd(float x, float y, float rad) {
    return gen_approach_cmd_intr(x, y, rad, &always_false);
}

inline Command gen_move_to_cmd_intr(float x, float y, char (*interrupt_condition)(oi_t * sensor_data)) {
    return gen_approach_cmd_intr(x, y, 0, interrupt_condition);
}
inline Command gen_move_to_cmd(float x, float y) {
    return gen_move_to_cmd_intr(x, y, &always_false);
}

// gen rotate command
inline Command gen_rotate_cmd_intr(float angle, char (*interrupt_condition)(oi_t * sensor_data)) {
    CommandData cd;
    cd.move = (MoveCD) {angle};

    Command c;
    c.on_start = &start_rotate_move; // updated for rotate
    c.is_complete = &move_end_cond;
    c.is_interrupt = interrupt_condition;
    c.data = cd;

    return c;
}
inline Command gen_rotate_cmd(float angle) {
    return gen_rotate_cmd_intr(angle, &always_false);
}

inline Command gen_rotate_to_cmd_intr(float angle, char (*interrupt_condition)(oi_t * sensor_data)) {
    CommandData cd;
    cd.move = (MoveCD) {angle};

    Command c;
    c.on_start = &start_rotate_move_to; // updated for rotate
    c.is_complete = &move_end_cond;
    c.is_interrupt = interrupt_condition;
    c.data = cd;

    return c;
}
inline Command gen_rotate_to_cmd(float angle) {
    return gen_rotate_to_cmd_intr(angle, &always_false);
}
