#include "console.h"

#include "rtt.h"

#define FRAME_MAGIC 0x4f46u

typedef struct {
    uint16_t magic;
    uint8_t stimulus, periods;
    uint16_t fault, angle_start, angle_end, motion_mrad;
    CurrentPulseResponse response;
} PulsePacket;

_Static_assert(sizeof(ConsoleStatus) == 60, "RTT layout");
_Static_assert(sizeof(ConsoleTrace) == 72, "trace layout");
_Static_assert(sizeof(PulsePacket) == 96, "pulse packet layout");

static bool parse_integer(const char **cursor, int32_t *value) {
    const char *p = *cursor;
    bool negative = *p == '-';
    p += negative;
    if (*p < '0' || *p > '9') return false;
    uint32_t magnitude = 0;
    while (*p >= '0' && *p <= '9') {
        uint32_t digit = (uint32_t)(*p++ - '0');
        if (magnitude > 214748364u || (magnitude == 214748364u && digit > (negative ? 8u : 7u))) {
            return false;
        }
        magnitude = magnitude * 10u + digit;
    }
    *cursor = p;
    *value = negative && magnitude == 2147483648u
        ? INT32_MIN : (negative ? -(int32_t)magnitude : (int32_t)magnitude);
    return true;
}

static bool decode(const char *text, ConsoleCommand *command) {
    if (*text < 'A' || *text > 'Z') return false;
    command->code = *text++;
    while (*text == ',' && command->count < CONSOLE_MAX_ARGS) {
        ++text;
        if (!parse_integer(&text, &command->args[command->count++])) return false;
    }
    return *text == '\0';
}

bool Console_Poll(ConsoleCommand *command) {
    static char input[40];
    static uint8_t used;
    static bool receiving;
    static bool overflow;
    char byte;
    for (unsigned i = 0; i < 16u && Rtt_ReadByte(&byte); ++i) {
        if (byte == '@') {
            receiving = true;
            overflow = false;
            used = 0;
        } else if (receiving && byte == ';') {
            receiving = false;
            input[used] = '\0';
            *command = (ConsoleCommand){.code = '?'};
            command->valid = !overflow && decode(input, command);
            return true;
        } else if (receiving) {
            if (used < sizeof(input) - 1u && byte != '\0') {
                input[used++] = byte;
            } else {
                overflow = true;
            }
        }
    }
    return false;
}

void Console_Report(ConsoleStatus *status) {
    status->magic = FRAME_MAGIC;
    status->version = 2;
    status->size = sizeof(*status);
    Rtt_Write(0, status, sizeof(*status));
}

void Console_Trace(const ConsoleTrace *sample) {
    Rtt_Write(1, sample, sizeof(*sample));
}

void Console_Pulse(uint8_t stimulus, uint8_t periods, uint16_t fault, uint16_t angle_start,
                   uint16_t angle_end, uint16_t motion_mrad, const CurrentPulseResponse *response) {
    PulsePacket packet = {0x4a46u, stimulus, periods, fault, angle_start, angle_end, motion_mrad, *response};
    Rtt_Write(1, &packet, sizeof(packet));
}
