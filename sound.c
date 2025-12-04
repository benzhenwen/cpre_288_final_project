#include "sound.h"

// Define songs
void sound_init() {
    // Song 0: Quick beep (1 short note)
    unsigned char beep_notes[] = {72};  // middle C note
    unsigned char beep_duration[] = {10};
    oi_loadSong(0, 1, beep_notes, beep_duration);

    // Song 1: Startup (2 notes)
    unsigned char startup_notes[] = {60, 67};   // C, G
    unsigned char startup_duration[] = {15, 15};
    oi_loadSong(1, 2, startup_notes, startup_duration);

    // Song 2: Success (3 notes)
    unsigned char success_notes[] = {60, 64, 67};       // C, E, G
    unsigned char success_duration[] = {8, 8, 12};
    oi_loadSong(2, 3, success_notes, success_duration);
}

void sound_beep() {
    oi_play_song(0);
}

void sound_startup() {
    oi_play_song(1);
}

void sound_success() {
    oi_play_song(2);
}
