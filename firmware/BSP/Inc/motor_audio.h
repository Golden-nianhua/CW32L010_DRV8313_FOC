#ifndef MOTOR_AUDIO_H
#define MOTOR_AUDIO_H
#include <stdint.h>
#include <stdbool.h>

enum { MOTOR_AUDIO_STATE_WORDS = 16, MOTOR_AUDIO_RATE = 48000 };
void MotorAudio_Begin(void *storage, uint32_t frames, bool mulaw);
void MotorAudio_End(void);
void MotorAudio_SetTargets(const int32_t base_q7[3], const int32_t gain_q7[3]);
void MotorAudio_Tick(void);
uint32_t MotorAudio_Frames(void);
uint32_t MotorAudio_Underruns(void);
#endif
