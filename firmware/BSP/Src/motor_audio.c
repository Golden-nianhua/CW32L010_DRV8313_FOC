#include "motor_audio.h"
#include "drv8313.h"
#include "rtt.h"
#include "cw32l010.h"
#include <string.h>

typedef struct {
    int32_t base[3], gain[3], voltage[3];
    uint16_t fraction[3];
    int16_t sample;
    uint32_t total;
    volatile uint32_t frames, underruns;
    bool started, starved, mulaw;
} AudioState;
_Static_assert(sizeof(AudioState) <= MOTOR_AUDIO_STATE_WORDS * 4, "audio arena");
static AudioState *audio;

void MotorAudio_Begin(void *storage, uint32_t frames, bool mulaw) {
    audio = storage;
    memset(audio, 0, sizeof(*audio));
    audio->total = frames;
    audio->mulaw = mulaw;
    audio->base[0] = audio->base[1] = audio->base[2] = 32000;
    Rtt_AudioBegin();
}

void MotorAudio_End(void) {
    audio = 0;
    Rtt_AudioEnd();
}

void MotorAudio_SetTargets(const int32_t base_q7[3], const int32_t gain_q7[3]) {
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    for (unsigned i = 0; i < 3; ++i) {
        audio->base[i] = base_q7[i];
        audio->gain[i] = gain_q7[i];
    }
    __set_PRIMASK(mask);
}

void MotorAudio_Tick(void) {
    AudioState *state = audio;
    if (!state) return;
    uint32_t pending_bytes = (state->total - state->frames) * (state->mulaw ? 1u : 2u);
    uint32_t prefill = pending_bytes < 256u ? pending_bytes : 256u;
    if (!state->started && pending_bytes && Rtt_AudioAvailable() >= prefill) {
        state->started = true;
        state->starved = false;
    }
    int16_t next;
    if (state->started && state->frames < state->total && Rtt_ReadAudio(&next, state->mulaw)) {
        state->sample = next;
        ++state->frames;
    } else {
        if (state->started && state->frames < state->total && !state->starved) {
            ++state->underruns;
            state->starved = true;
            state->started = false;
        }
        state->sample -= state->sample / 16;
        if (state->sample > -16 && state->sample < 16) state->sample = 0;
    }
    for (unsigned i = 0; i < 3; ++i) {
        state->voltage[i] = state->sample * state->gain[i] >> 15;
    }
    uint16_t duty[3];
    for (unsigned i = 0; i < 3; ++i) {
        int32_t value = state->base[i] + state->voltage[i];
        if (value < 0) value = 0;
        if (value > 64000) value = 64000;
        uint32_t accumulator = state->fraction[i] + (uint32_t)value;
        duty[i] = (uint16_t)(accumulator >> 7);
        state->fraction[i] = (uint16_t)(accumulator & 127u);
    }
    DRV8313_SetDuty(duty[0], duty[1], duty[2]);
}

uint32_t MotorAudio_Frames(void) { return audio ? audio->frames : 0; }
uint32_t MotorAudio_Underruns(void) { return audio ? audio->underruns : 0; }
