#include <stdlib.h>
#include "open_interface.h"
#include "movement.h"
#include "command.h"
#include "movementcommands.h"
#include "scan.h"
#include "uart.h"
#include "button.h"
#include "Timer.h"
#include "data_protocol.h"
#include "ir.h"
#include "ping.h"
#include "servo.h"



// cal values local store for servo


// for bot 1: 7050, 35100
// for bot 3: 9750, 39900
// for bot 5: 8400, 35700
// for bot 7: 8200, 35900
// for bot 17: 7050, 34050

#define CAL_A 8400
#define CAL_B 35700


// cal values local store for ir
// for bot 1: 41034.980469, 0.011351
// for bot 3:
// for bot 5: 36906.015625, 2.875138
// for bot 7: 37082.558594, 3.035430
// for bot 17: 14464.520508, 7.239097

#define CAL_IR_A 36906.015625
#define CAL_IR_B 2.875138


// ---------------- SCAN DATA ----------------
#include "main_scan_data.h"


// ---------------- OBJECT ALGS ----------------
#include "main_objects.h"


// ---------------- PATHING ALGS ----------------
#include "main_pathfinding.h"


// ---------------- AUTO PATH MOVE ----------------
#include "main_auto_move.h"


// ---------------- IR AUTOCAL ----------------
#include "main_ir_autocal.h"









int main(void)
{
    timer_init();
    lcd_init();
    cq_oi_init();
    ur_init();
    ur_inter_init();


    pn_init();
    ir_init_fuck();
    ir_set_a_b(CAL_IR_A, CAL_IR_B);
    sv_init();
    sv_set_cal_known(CAL_A, CAL_B);

    lcd_printf("meow");

    ur_send_line("-------------start--------------");


    // send some inital data
    send_data_packet(object_map, object_map_c); // update python data packet


    // MAIN LOOP
    while(1) {

        // ---------- CHECKS FOR NEW COMMANDS ----------
        if (ur_intr_line_ready()) {

            // copy the command
            char command[64];
            strcpy(command, ur_intr_get_line());

            // early termination
            if (command[0] == 'e') {
                break;
            }
            // terminate all active commands
            else if (command[0] == 'k') {
                cq_clear();
                move_stop();
                send_data_packet(object_map, object_map_c); // update python data packet
            }
            // run a scan
            else if (command[0] == 's') {
                // object scan
                perform_scan_and_obj_detection();
                send_data_packet(object_map, object_map_c); // update python data packet
            }
            // start auto mode
            else if (command[0] == 'a' || command[0] == 'p') {
                // full auto mode
                full_auto_flag = command[0] == 'p';

                // object scan
                perform_scan_and_obj_detection();
                send_data_packet(object_map, object_map_c); // update python data packet

                // move to smallest
                int smallest_index = find_smallest_object_index();
                if (smallest_index != -1) {
                    // error is bot size + smallest object size
                    cq_queue(gen_approach_cmd_intr(object_map[smallest_index].x, object_map[smallest_index].y, 160 + (object_map[smallest_index].radius), &move_bump_interrupt_callback));
                }
            }
            else if (command[0] == 'c') { // servo cal
                sv_cal();
            }
            else if (command[0] == 'i') { // ir cal auto
                ir_auto_cal_init();
                sc_point_servo(90);

                ir_auto_cal_step = 0;
                cq_queue(gen_move_reverse_cmd(100));

                CommandData cd;
                cd.move = (MoveCD) {0};

                Command c;
                c.on_start = &start_rotate_move; // updated for rotate
                c.is_complete = &auto_cal_end_callback;
                c.is_interrupt = &always_false;
                c.data = cd;

                cq_queue(c);
            }
            else if (command[0] == '*') {
                float d = pb_get_dist();

                char buff[32];
                sprintf(buff, "ping dist: %.5f", d);
                ur_send_line(buff);
            }
            else if (command[0] == '!') {
                object_map_c = 0;
                reset_pos();
                send_data_packet(object_map, object_map_c); // update python data packet
            }

            // is a non special command that has a second part
            else {
                // parse integer part
                int instruction_value;
                sscanf(&command[1], "%d", &instruction_value);

                if      (command[0] == 'f') cq_queue(gen_move_cmd_intr(instruction_value, &move_bump_interrupt_callback)); // allow bot to bump and auto detect
                else if (command[0] == 'r') cq_queue(gen_move_reverse_cmd(instruction_value));
                else if (command[0] == 't') {
                    move_stop();
                    cq_clear();
                    cq_queue(gen_rotate_cmd(instruction_value));
                }
                else if (command[0] == 'm') {
                    move_stop();

                    float x = instruction_value % 10000 - 5000;
                    float y = instruction_value / 10000 - 5000;

                    char buff[32];
                    sprintf(buff, "click move to: (%.0f, %.0f)", x, y);
                    ur_send_line(buff);

                    cq_clear();
                    cq_queue(gen_move_to_cmd_intr(x, y, &move_bump_interrupt_callback));
                }
            }
        }




        // standard main loop call, update commands
        cq_update();
        if (cq_size() > 0) {
//            lcd_printf("(%.1f, %.1f, %.1f)\nqs: %d\n(%.1f, %.1f, %.1f)", get_pos_x(), get_pos_y(), get_pos_r(), cq_size(), get_target_x(), get_target_y(), get_target_r());
            send_data_packet(object_map, object_map_c); // update python data packet
        }

    }

    // end
    ur_send_line("done");

    cq_oi_free();

    return 0;
}

