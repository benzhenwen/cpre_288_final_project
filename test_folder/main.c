#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void ur_send_line(char * str) {
    printf("uart_line: %s\n", str);
}

#include "main_scan_data.h"
#include "main_pathfinding.h"

#include "main_pathfinding_new.h"

void add_object(object_positional object) {
    object_map[object_map_c] = object;
    object_map_c++;
}
    
// position stuff
float pos_x = 0;
float pos_y = 0;
float pos_r = 0;

float get_pos_x() { return pos_x; }
float get_pos_y() { return pos_y; }
float get_pos_r() { return pos_r; }

void move_to(float x, float y) {
    pos_x = x;
    pos_y = y;
}

void print_objs_as_desmos() {
    int i;
    for (i = 0; i < object_map_c; i++) {
        object_positional o = object_map[i];
        printf("\\left(x-%.2f\\right)^{2}+\\left(y-%.2f\\right)^{2}=%.2f^{2}\n", o.x, o.y, o.radius);
    }
}



int main() {
    printf("meow\n");

    add_object((object_positional) {500, 0, 40, 1});
    print_objs_as_desmos();

    exp_map_new_searched_point(4 * 333, 3 * 333);
    printf("%d\n", exp_map_val_at(4, 3));

}