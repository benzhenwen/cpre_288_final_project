#pragma once

// init gpio and timer 3b for ping sensor
void pn_init();

// returns the float dist of the ping and clears ping_ready flag
float pb_get_dist();
