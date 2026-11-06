#ifndef HFI_H
#define HFI_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int32_t x, y;
    int32_t angle_q8;
    int32_t velocity_q8;
    bool angle_override;
    uint16_t frames;
    uint8_t weak_frames;
} HfiObserver;

/* One frame contains forty 25 kHz carrier cycles, averaged in Q3 ADC counts. */
bool Hfi_Update(HfiObserver *observer, int16_t delta[4][2], bool locking);
uint16_t Hfi_Angle(const HfiObserver *observer);
int32_t Hfi_Speed(const HfiObserver *observer);
void Hfi_SetAngle(HfiObserver *observer, uint16_t phase);
void Hfi_ReleaseAngle(HfiObserver *observer, uint16_t phase, int32_t speed_q8);

#endif
