#ifndef CONSOLE_H
#define CONSOLE_H

#include <stdbool.h>
#include <stdint.h>
#include "current_adc.h"

enum { CONSOLE_MAX_ARGS = 3 };

typedef struct {
    char code;
    uint8_t count;
    bool valid;
    int32_t args[CONSOLE_MAX_ARGS];
} ConsoleCommand;

typedef struct {
    uint16_t magic;
    uint8_t version;
    uint8_t size;
    uint8_t mode;
    uint8_t state;
    uint8_t event;
    uint8_t command;
    uint8_t result;
    uint8_t control_source;
    uint16_t ack;
    int32_t target;
    int32_t position;
    int32_t speed;
    int32_t id;
    int32_t iq;
    int32_t iq_ref;
    int32_t vd;
    int32_t vq;
    int32_t profile;
    uint32_t missed_samples;
    uint32_t encoder_errors;
    uint32_t sample_us;
    uint16_t bus_mv;
    uint16_t voltage_limit_mv;
    uint16_t supply_raw;
    uint8_t bus_source;
    uint8_t key_pressed;
} ConsoleStatus;

typedef struct {
    uint32_t sequence;
    uint32_t time_us;
    int32_t speed;
    int32_t hfi_speed;
    uint16_t raw_angle;
    uint16_t electrical_angle;
    uint16_t hfi_angle;
    uint16_t adc_b;
    uint16_t adc_c;
    int16_t id;
    int16_t iq;
    int16_t iq_ref;
    int16_t vd;
    int16_t vq;
    uint8_t observer_flags;
    uint8_t control_source;
    uint16_t index;
    int32_t smo_speed_q8;
    uint16_t smo_angle;
    uint16_t smo_emf;
    uint16_t smo_error;
    uint16_t smo_sample_us;
    uint32_t hfi_sequence;
    uint32_t hfi_missed_frames;
    uint16_t control_angle;
    uint16_t injection_mv;
    int16_t observer_voltage_alpha;
    int16_t observer_voltage_beta;
    int16_t observer_current_alpha;
    int16_t observer_current_beta;
    int16_t id_ref;
    uint16_t iq_limit;
    int16_t weakening_target;
    uint16_t voltage_magnitude;
} ConsoleTrace;

bool Console_Poll(ConsoleCommand *command);
void Console_Report(ConsoleStatus *status);
void Console_Trace(const ConsoleTrace *sample);
void Console_Pulse(uint8_t stimulus, uint8_t periods, uint16_t fault, uint16_t angle_start,
                   uint16_t angle_end, uint16_t motion_mrad, const CurrentPulseResponse *response);

#endif
