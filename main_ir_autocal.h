#pragma once



#include "Timer.h"
#include "ir.h"
#include "ping.h"
#include "servo.h"


// for auto calibration of ir sequence
// step 0 is at 10cm, step 1 at 15cm, step 2 at 20cm, etc... step 8
int ir_auto_cal_step = 0;

char auto_cal_end_callback(oi_t * sensor_data) {
    char end_cond = move_end_cond(sensor_data);

    if (end_cond) {
        // collect that point
        int ir_scan;
        do {
            timer_waitMillis(100);
            ir_scan = sc_scan_ir(90);

            char buff[32];
            sprintf(buff, "(attempt) ir: %d", ir_scan);
            ur_send_line(buff);

        } while (ir_scan < 100 || (ir_auto_cal_step == 0 && ir_scan < 1000));

        float ping_scan = pb_get_dist();

        ir_auto_cal_add_point(ir_scan, ping_scan);

        char buff[64];
        sprintf(buff, "autocal point - ir: %d, ping: %.6f", ir_scan, ping_scan);
        ur_send_line(buff);


        // check end cond
        if (++ir_auto_cal_step >= 10) {
            Curve output = ir_auto_cal_calculate();

            ir_set_a_b(output.a, output.b);

            char buff[64];
            sprintf(buff, "autocal values - a: %.6f, b: %.6f", output.a, output.b);
            ur_send_line(buff);

            return end_cond;
        }


        // queue the next reverse command
        cq_queue(gen_move_reverse_cmd(50));

        CommandData cd;
        cd.move = (MoveCD) {0};

        Command c;
        c.on_start = &start_rotate_move; // updated for rotate
        c.is_complete = &auto_cal_end_callback;
        c.is_interrupt = &always_false;
        c.data = cd;

        cq_queue(c);
    }

    return end_cond;
}
