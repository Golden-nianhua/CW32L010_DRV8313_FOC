#include "rtt.h"
#include "cw32l010.h"
#include <string.h>

#define RTT_UP_SIZE 128u
#define RTT_TRACE_SIZE 512u
#define RTT_DOWN_SIZE 32u

typedef struct {
    const char *name;
    volatile uint8_t *data;
    uint32_t size;
    volatile uint32_t write_offset;
    volatile uint32_t read_offset;
    uint32_t flags;
} RttChannel;

typedef struct {
    char id[16];
    uint32_t up_count;
    uint32_t down_count;
    RttChannel up[2];
    RttChannel down[2];
} RttControlBlock;

/* Publish ordinary RAM payloads through the volatile write offset after DMB. */
_Alignas(4) static uint8_t up_data[RTT_UP_SIZE];
_Alignas(4) static uint8_t trace_data[RTT_TRACE_SIZE];
static volatile uint8_t down_data[RTT_DOWN_SIZE];

/* SEGGER RTT layout for 32-bit targets. */
RttControlBlock _SEGGER_RTT = {
    .id = "SEGGER RTT",
    .up_count = 2,
    .down_count = 2,
    .up = {
        {.name = "Terminal", .data = up_data, .size = RTT_UP_SIZE},
        {.name = "FOC trace", .data = trace_data, .size = RTT_TRACE_SIZE},
    },
    .down = {
        {.name = "Terminal", .data = down_data, .size = RTT_DOWN_SIZE},
        {.name = "PCM16", .data = trace_data, .size = RTT_TRACE_SIZE},
    },
};

static void copy_bytes(uint8_t *destination, const uint8_t *source, size_t length) {
    if (((uintptr_t)destination | (uintptr_t)source) % 4u == 0) {
        while (length >= 4u) {
            memcpy(__builtin_assume_aligned(destination, 4),
                   __builtin_assume_aligned(source, 4), 4);
            destination += 4;
            source += 4;
            length -= 4;
        }
    }
    while (length-- != 0) *destination++ = *source++;
}

static bool audio_owns_trace;

void Rtt_Write(unsigned channel, const void *data, size_t length) {
    if (channel == 1 && audio_owns_trace) return;
    RttChannel *up = &_SEGGER_RTT.up[channel];
    uint32_t mask = up->size - 1u;
    uint32_t offset = up->write_offset;
    uint32_t free_bytes = (up->read_offset - offset - 1u) & mask;
    if (length > free_bytes) {
        return;
    }

    size_t first = up->size - offset;
    if (first > length) first = length;
    copy_bytes((uint8_t *)up->data + offset, data, first);
    copy_bytes((uint8_t *)up->data, (const uint8_t *)data + first, length - first);
    __DMB();
    up->write_offset = (offset + length) & mask;
}

bool Rtt_ReadByte(char *byte) {
    uint32_t offset = _SEGGER_RTT.down[0].read_offset;
    if (offset == _SEGGER_RTT.down[0].write_offset) {
        return false;
    }

    __DMB();
    *byte = down_data[offset];
    __DMB();
    _SEGGER_RTT.down[0].read_offset = (offset + 1u) & (RTT_DOWN_SIZE - 1u);
    return true;
}

void Rtt_AudioBegin(void) {
    _SEGGER_RTT.down[1].read_offset = _SEGGER_RTT.down[1].write_offset = 0;
    audio_owns_trace = true;
}
void Rtt_AudioEnd(void) { audio_owns_trace = false; }
uint32_t Rtt_AudioAvailable(void) {
    return (_SEGGER_RTT.down[1].write_offset - _SEGGER_RTT.down[1].read_offset) & (RTT_TRACE_SIZE - 1u);
}
bool Rtt_ReadAudio(int16_t *sample, bool mulaw) {
    uint32_t bytes = mulaw ? 1u : 2u;
    if (Rtt_AudioAvailable() < bytes) return false;
    uint32_t offset = _SEGGER_RTT.down[1].read_offset;
    __DMB();
    if (mulaw) {
        unsigned code = (unsigned)(~trace_data[offset]) & 255u;
        int32_t magnitude = (((code & 15u) << 3) + 132u) << ((code >> 4) & 7u);
        *sample = (int16_t)((code & 128u) ? 132 - magnitude : magnitude - 132);
    } else {
        *sample = (int16_t)(trace_data[offset] | ((uint16_t)trace_data[(offset + 1u) & (RTT_TRACE_SIZE - 1u)] << 8));
    }
    __DMB();
    _SEGGER_RTT.down[1].read_offset = (offset + bytes) & (RTT_TRACE_SIZE - 1u);
    return true;
}
