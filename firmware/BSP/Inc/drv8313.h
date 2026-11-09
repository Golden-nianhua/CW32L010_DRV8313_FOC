#ifndef DRV8313_H
#define DRV8313_H

#include <stdbool.h>
#include <stdint.h>

#define DRV8313_PWM_MAX 1199u
#define DRV8313_HFI_PWM_MAX 239u

void DRV8313_Init(void);
void DRV8313_SetEnabled(bool enabled);
void DRV8313_SetHfiTiming(bool enabled);
void DRV8313_SetAudioTiming(bool enabled);
void DRV8313_SetDuty(uint16_t a, uint16_t b, uint16_t c);

#endif
