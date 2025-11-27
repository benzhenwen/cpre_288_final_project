#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void ur_send_line(char * str) {
    printf("uart_line: %s\n", str);
}

// position stuff
float pos_x = 0;
float pos_y = 0;
float pos_r = 0;

float get_pos_x() { return pos_x; }
float get_pos_y() { return pos_y; }
float get_pos_r() { return pos_r; }

#include "main_scan_data.h"
#include "main_pathfinding.h"

#include "main_pathfinding_new.h"

void add_object(object_positional object) {
    object_map[object_map_c] = object;
    object_map_c++;
}

void move_to(float x, float y) {
    pos_x = x;
    pos_y = y;
}

void print_objs_as_desmos() {
    int i;
    for (i = 0; i < object_map_c; i++) {
        object_positional o = object_map[i];
        printf("\\left(x-%.2f\\right)^{2}+\\left(y-%.2f\\right)^{2}=%.2f^{2}\n", o.x, o.y, o.radius + 170);
    }
}



int main() {
    printf("meow\n");

    // add_object((object_positional) {-99999, 0, 99999, 1});
    // add_object((object_positional) {0, -99999, 99999, 1});
    add_object((object_positional) {230, 0, 40, 1});
    add_object((object_positional) {500, 100, 40, 1});
    add_object((object_positional) {300, -300, 40, 1});
    print_objs_as_desmos();

    float ox, oy;
    path_to(0, 0, 1000, 0, &ox, &oy);
    printf("(%.2f, %.2f)\n", ox, oy);

    // int i, x, y;
    // for (i = 0; i < 16; i++) {
    //     for (x = 0; x < 16; x++) for (y = 0; y < 16; y++) {
    //         if (x == 5);
    //         else exp_map_new_searched_point(x * 333.333f, y * 333.333f); 
    //     }
    // }

    // printf("%d\n", exp_map_val_at(2, 1));
    
    // for (i = 0; i < 20; i++) {
    //     float px, py;
    //     exp_map_pick_random_point(&px, &py);
    //     printf("(%.2f, %.2f)\n", px, py);
    // }

}