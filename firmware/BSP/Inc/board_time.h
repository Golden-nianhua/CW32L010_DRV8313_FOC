#ifndef BOARD_TIME_H
#define BOARD_TIME_H

#include <stdint.h>

#define BOARD_CLOCK_HZ 48000000u

void BoardTime_InitClock(void);
void BoardTime_Start(void);
uint32_t BoardTime_NowUs(void);
void BoardTime_DelayMs(uint32_t milliseconds);

#endif
