#pragma once

// init gpio and timer 3b for ping sensor
void pn_init();

// performs a ping. blocking. returns the float dist
float pb_get_dist();
