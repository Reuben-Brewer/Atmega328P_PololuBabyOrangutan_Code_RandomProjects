#ifndef _LC_MIDI_IN_C_H_
#define _LC_MIDI_IN_C_H_
#include <stdint.h>

unsigned int numberMidiNotes_LC = 17;

unsigned int noteList_startTime_MidiTicks_LC[] = {96, 192, 288, 384, 672, 864, 960, 1056, 1152, 1440, 2016, 2208, 2304, 2400, 2496, 2592, 2688};

char noteList_pitchMidiNumber_LC[] = {67, 67, 67, 72, 76, 67, 67, 67, 72, 76, 72, 72, 71, 71, 69, 69, 67};

#endif
