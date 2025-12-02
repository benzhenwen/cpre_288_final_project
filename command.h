#pragma once

#include "open_interface.h"


// datas for commands
typedef struct MoveCD {
    float distance;
} MoveCD;

typedef struct MoveToCD {
    float x;
    float y;
    float apprach_rad;
} MoveToCD;

typedef struct FunctionPointerCD {
    void (*function)();
} FunctionPointerCD;

typedef union CommandData {
    MoveCD move;
    MoveToCD moveTo;
    FunctionPointerCD functionPointer;
} CommandData;


typedef struct Command {
    // function to be called when command is started
    void (*on_start)(CommandData * data);

    // should return 0 when running, and not 0 when done. the very first time this function returns not 0 the queue will advance, and is_complete will never be queried again.
    // commands are expected to end themselves.
    char (*is_complete)(oi_t * sensor_data);

    // should return not 0 when we want to interrupt the command. the very first time this function returns not 0 the queue will advance, and is_interrupt will never be queried again
    // before is_interrupt returns true, this function should perform any interrupt work first.
    char (*is_interrupt)(oi_t * sensor_data);

    // command data
    CommandData data;

} Command;


inline char always_false(oi_t * sensor_data) {
    return false;
}
inline char always_true(oi_t * sensor_data) {
    return true;
}


// oi stuff init
void cq_oi_init();

// free - call at end
void cq_oi_free();


// add a command to the queue, returns -1 if overflow
int cq_queue(Command com);

// get the queue size
int cq_size();

// get the pointer to the top command on the queue, or NULL if the queue is empty!!
Command* cq_top();

// resets the command queue
void cq_clear();

// main update function, called once per while loop
void cq_update();
