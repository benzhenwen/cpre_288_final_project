#include "open_interface.h"
#include "command.h"

#include "movement.h"

#include "uart.h"


// ----------------------------- core projet - sensor data -----------------------------
oi_t * sensor_data;

// init - call at start
void cq_oi_init() {
    sensor_data = oi_alloc();
    oi_init(sensor_data);
}

// free - call at end
void cq_oi_free() {
    oi_free(sensor_data);
}





// command queue
#define COMMAND_QUEUE_SIZE 8

Command command_queue[COMMAND_QUEUE_SIZE];
int queue_top_index = 0;
int queue_write_index = 0;
int queue_size = 0;

int command_active = 0;

// to queue a command, returns number of commands in queue, or -1 on fail
int cq_queue(Command com) {
    if (queue_size >= COMMAND_QUEUE_SIZE) return -1; // queue is full

    command_queue[queue_write_index] = com; // write to the queue

    // update trackers
    queue_write_index++;
    if (queue_write_index >= COMMAND_QUEUE_SIZE) queue_write_index = 0;
    queue_size++;

    return queue_size;
}

int cq_queue_front(Command com) {
    if (queue_size >= COMMAND_QUEUE_SIZE) return -1; // queue is full

    int i = queue_write_index;
    while (i != queue_top_index) {
        command_queue[i] = command_queue[(i - 1 < 0) ? (COMMAND_QUEUE_SIZE - 1) : (i - 1)];
        i--;
        if (i < 0) i = COMMAND_QUEUE_SIZE - 1;
    }

    if (command_active) {
        // place the command at next, we are assuming cq_queue_front is being called only once at the tail end of a interrupt callback
        // this prevents the about to end command from being shifted one and causing it to be repeatedly called
        command_queue[(queue_top_index + 1 >= COMMAND_QUEUE_SIZE) ? 0 : (queue_top_index + 1)] = com;
    }
    else command_queue[queue_top_index] = com; // normal behavior, place at front

    queue_write_index++;
    if (queue_write_index >= COMMAND_QUEUE_SIZE) queue_write_index = 0;
    queue_size++;

    return queue_size;
}

// get the queue size
int cq_size() {
    return queue_size;
}

// get the top command
Command* cq_top() {
    if (queue_size == 0) return NULL;
    return &command_queue[queue_top_index];
}

// advance the command queue
void cq_next() {
    queue_size--;
    queue_top_index++;
    if (queue_top_index >= COMMAND_QUEUE_SIZE) queue_top_index = 0;
}

// resets the command queue
void cq_clear() {
    queue_top_index = 0;
    queue_write_index = 0;
    queue_size = 0;
    command_active = 0;
}



// main update function, called once per while loop
void cq_update() {
    if (cq_size() == 0) return; // do nothing if queue empty

    // updaoi_updatete sensor data
    oi_update(sensor_data);

    // update movement data
    update_position_data(sensor_data);


    // fetch current command
    Command * cmd = cq_top();

    // process new command if none running
    if (!command_active) {
        command_active = 1;

        ur_send_line("command starting");

        cmd->on_start(&cmd->data); // start the command
    }

    // process already running command
    else {
        if(cmd->is_complete(sensor_data) || cmd->is_interrupt(sensor_data)) { // if the command is complete, move to the next command
            cq_next();
            command_active = 0;

            ur_send_line("command ended");
        }
    }

}
