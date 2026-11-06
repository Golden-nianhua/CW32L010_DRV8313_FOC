#include "current_adc.h"

#include <string.h>
#include "drv8313.h"
#include "board_time.h"
#include "cw32l010_adc.h"
#include "cw32l010_gpio.h"
#include "cw32l010_sysctrl.h"

static volatile uint32_t sequence;
typedef struct {
    uint16_t b, c;
    uint32_t time_us;
} CurrentSample;
enum { SAMPLE_QUEUE_SIZE = 8 };
static CurrentSample samples[SAMPLE_QUEUE_SIZE];
static uint32_t consumed_sequence;
static uint8_t pwm_periods;
static uint8_t control_divider = 20;

enum {
    PULSE_GAP = 8,
    PULSE_MAX_CURRENT_COUNTS = 155,
    HFI_MAX_CURRENT_MA_DEFAULT = 2000,
    HFI_SETTLE_CYCLES = 2,
    HFI_MEASURE_CYCLES = CURRENT_HFI_CONTROL_CYCLES - HFI_SETTLE_CYCLES,
};
_Static_assert(HFI_MEASURE_CYCLES == 38, "update response reciprocals when frame length changes");
static volatile int32_t hfi_max_current_counts = HFI_MAX_CURRENT_MA_DEFAULT * 2048 / 3300;
static struct {
    uint16_t duty[CURRENT_PULSE_VECTORS + 1][3];
    uint16_t zero_b, zero_c;
    union {
        struct {
            uint16_t base_b, base_c;
        };
        uint8_t duty_fraction[4];
    };
    uint16_t fault;
    uint8_t periods, repeats, vector, wait, point;
    bool rotating;
    bool continuous;
    bool fast_smo;
    volatile bool active;
} pulse;

typedef union {
    struct {
        int32_t sum[CURRENT_PULSE_VECTORS][2];
        int16_t edge[2];
        int32_t voltage_sum[2];
    };
    uint32_t words[11];
} HfiAccumulation;

typedef struct {
    uint16_t duty[2][CURRENT_PULSE_VECTORS + 1][3];
    uint16_t (*volatile next_duty)[3];
    HfiAccumulation accumulation[2];
    volatile uint32_t sequence;
    HfiAccumulation *volatile writing;
} HfiState;

/* Finite calibration and continuous control own this storage exclusively.
 * Starting either operation clears its member before enabling the ADC path. */
static union {
    CurrentPulseResponse response[CURRENT_PULSE_VECTORS];
    HfiState hfi;
} measurement;

/* At 2 A, 38 samples reach 47158 counts per vector. Keep accumulation in
 * 32 bits. These Q18 reciprocals give a quotient at most one count too large. */
static int32_t response_divide(int32_t value, uint32_t divisor, uint32_t reciprocal) {
    uint32_t magnitude = (uint32_t)(value < 0 ? -value : value);
    uint32_t quotient = magnitude * reciprocal >> 18;
    if (quotient * divisor > magnitude) --quotient;
    return value < 0 ? -(int32_t)quotient : (int32_t)quotient;
}

static void hfi_duties(uint16_t dest[CURRENT_PULSE_VECTORS + 1][3],
                       uint16_t source[CURRENT_PULSE_VECTORS + 1][3]) {
    for (unsigned i = 0; i <= CURRENT_PULSE_VECTORS; ++i) {
        for (unsigned j = 0; j < 3; ++j) {
            dest[i][j] = (uint16_t)((source[i][j] + 2u) * 52429u >> 18);
        }
    }
}

static void hfi_targets(uint16_t dest[CURRENT_PULSE_VECTORS + 1][3],
                        uint16_t source[CURRENT_PULSE_VECTORS + 1][3]) {
    for (unsigned phase = 0; phase < 3; ++phase) {
        dest[CURRENT_PULSE_VECTORS][phase] = source[CURRENT_PULSE_VECTORS][phase];
    }
    for (unsigned i = 0; i < CURRENT_PULSE_VECTORS; ++i) {
        for (unsigned phase = 0; phase < 3; ++phase) {
            int32_t vector = source[i][phase];
            dest[i][phase] = (uint16_t)(vector - dest[CURRENT_PULSE_VECTORS][phase]);
        }
    }
}

static inline __attribute__((always_inline)) uint16_t quantize_duty(int32_t value, unsigned phase) {
    int32_t quantized = value + pulse.duty_fraction[phase];
    if (quantized < 0) quantized = 0;
    if (quantized > (int32_t)(DRV8313_HFI_PWM_MAX + 1u) * 256) {
        quantized = (DRV8313_HFI_PWM_MAX + 1u) * 256;
    }
    pulse.duty_fraction[phase] = (uint8_t)quantized;
    return (uint16_t)(quantized >> 8);
}

__attribute__((section(".RamFunc"))) static void pulse_duty(unsigned index) {
    if (pulse.continuous && !pulse.fast_smo) {
        uint16_t *base = pulse.duty[CURRENT_PULSE_VECTORS];
        const uint16_t *target = measurement.hfi.next_duty[CURRENT_PULSE_VECTORS];
        if (index == 0) {
            for (unsigned phase = 0; phase < 3; ++phase) {
                base[phase] = (uint16_t)((7u * base[phase] + target[phase] + 4u) >> 3);
            }
        }
        const uint16_t *injection = measurement.hfi.next_duty[index];
        int32_t a = base[0], b = base[1], c = base[2];
        if (index < CURRENT_PULSE_VECTORS) {
            a += (int16_t)injection[0];
            b += (int16_t)injection[1];
            c += (int16_t)injection[2];
        }
        uint16_t duty_a = quantize_duty(a, 0);
        uint16_t duty_b = quantize_duty(b, 1);
        uint16_t duty_c = quantize_duty(c, 2);
        CW_ATIM->CR1_f.UDIS = 1;
        CW_ATIM->CCR1 = duty_a;
        CW_ATIM->CCR2 = duty_b;
        CW_ATIM->CCR4 = duty_c;
        CW_ATIM->CR1_f.UDIS = 0;
        return;
    }
    DRV8313_SetDuty(pulse.duty[index][0], pulse.duty[index][1], pulse.duty[index][2]);
}

__attribute__((noinline)) static void pulse_finish(uint16_t fault) {
    DRV8313_SetEnabled(false);
    pulse_duty(CURRENT_PULSE_VECTORS);
    if (pulse.rotating) {
        DRV8313_SetHfiTiming(false);
        control_divider = 20;
        pwm_periods = 0;
    }
    pulse.fault = fault;
    __DMB();
    pulse.active = false;
}

static void start(uint16_t duty[CURRENT_PULSE_VECTORS + 1][3], uint8_t periods,
                  uint8_t repeats, uint16_t zero_b, uint16_t zero_c, bool continuous) {
    if (continuous) hfi_targets(pulse.duty, duty);
    else if (periods == 0) hfi_duties(pulse.duty, duty);
    else memcpy(pulse.duty, duty, sizeof(pulse.duty));
    pulse.periods = periods;
    pulse.repeats = repeats;
    pulse.vector = 0;
    pulse.wait = PULSE_GAP;
    pulse.point = 0;
    pulse.zero_b = zero_b;
    pulse.zero_c = zero_c;
    pulse.fault = 0;
    pulse.continuous = continuous;
    pulse.fast_smo = false;
    pulse.rotating = periods == 0;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (pulse.rotating) {
        DRV8313_SetHfiTiming(true);
        control_divider = 200;
        pwm_periods = 0;
    }
    pulse_duty(CURRENT_PULSE_VECTORS);
    DRV8313_SetEnabled(true);
    pulse.active = true;
    __set_PRIMASK(mask);
}

void CurrentAdc_StartPulse(uint16_t duty[CURRENT_PULSE_VECTORS + 1][3], uint8_t periods,
                           uint8_t repeats, uint16_t zero_b, uint16_t zero_c) {
    memset(measurement.response, 0, sizeof(measurement.response));
    start(duty, periods, repeats, zero_b, zero_c, false);
}

void CurrentAdc_StartHfi(uint16_t duty[CURRENT_PULSE_VECTORS + 1][3], uint16_t zero_b, uint16_t zero_c) {
    memset(&measurement.hfi, 0, sizeof(measurement.hfi));
    measurement.hfi.writing = &measurement.hfi.accumulation[0];
    memset(pulse.duty_fraction, 0, sizeof(pulse.duty_fraction));
    measurement.hfi.next_duty = measurement.hfi.duty[0];
    hfi_targets(measurement.hfi.next_duty, duty);
    start(duty, 0, 0, zero_b, zero_c, true);
}

void CurrentAdc_SetHfiCurrentLimit(uint16_t current_ma) {
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    hfi_max_current_counts = (int32_t)current_ma * 2048 / 3300;
    __set_PRIMASK(mask);
}

void CurrentAdc_SetHfiDuty(uint16_t duty[CURRENT_PULSE_VECTORS + 1][3]) {
    uint16_t (*next)[3] = measurement.hfi.next_duty == measurement.hfi.duty[0] ? measurement.hfi.duty[1] : measurement.hfi.duty[0];
    hfi_targets(next, duty);
    __DMB();
    measurement.hfi.next_duty = next;
}

void CurrentAdc_SetSmoMode(bool enabled, uint16_t duty[CURRENT_PULSE_VECTORS + 1][3]) {
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    memset(measurement.hfi.accumulation, 0, sizeof(measurement.hfi.accumulation));
    measurement.hfi.writing = &measurement.hfi.accumulation[0];
    pulse.point = pulse.vector = pulse.wait = 0;
    pwm_periods = enabled ? (uint8_t)(pwm_periods / 5u) : (uint8_t)(pwm_periods * 5u);
    pulse.fast_smo = enabled;
    if (enabled) {
        control_divider = 40;
        for (unsigned phase = 0; phase < 3; ++phase) {
            uint32_t normal = (duty[CURRENT_PULSE_VECTORS][phase] * 5u + 128u) >> 8;
            pulse.duty[CURRENT_PULSE_VECTORS][phase] =
                (uint16_t)(normal > DRV8313_PWM_MAX ? DRV8313_PWM_MAX : normal);
        }
    } else {
        control_divider = 200;
        memset(pulse.duty_fraction, 0, sizeof(pulse.duty_fraction));
        hfi_targets(pulse.duty, duty);
        measurement.hfi.next_duty = measurement.hfi.duty[0];
        hfi_targets(measurement.hfi.next_duty, duty);
    }
    DRV8313_SetHfiTiming(!enabled);
    pulse_duty(CURRENT_PULSE_VECTORS);
    /* Discard an in-flight conversion from the previous PWM period. */
    ADC_ClearITPendingAll();
    NVIC_ClearPendingIRQ(ADC_IRQn);
    __set_PRIMASK(mask);
}

void CurrentAdc_SetSmoDuty(uint16_t a, uint16_t b, uint16_t c) {
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    uint16_t *base = pulse.duty[CURRENT_PULSE_VECTORS];
    base[0] = a;
    base[1] = b;
    base[2] = c;
    DRV8313_SetDuty(a, b, c);
    __set_PRIMASK(mask);
}

bool CurrentAdc_ReadHfi(CurrentHfiFrame *frame) {
    HfiAccumulation snapshot;
    uint32_t sequence;
    do {
        sequence = measurement.hfi.sequence;
        if (sequence == frame->sequence) return false;
        __DMB();
        const HfiAccumulation *published =
            measurement.hfi.writing == &measurement.hfi.accumulation[0]
                ? &measurement.hfi.accumulation[1] : &measurement.hfi.accumulation[0];
        memcpy(&snapshot, published, sizeof(snapshot));
        __DMB();
    } while (sequence != measurement.hfi.sequence);
    frame->sequence = sequence;
    if (pulse.fast_smo) {
        frame->sample_us = CURRENT_SMO_SAMPLE_US;
        memset(frame->delta, 0, sizeof(frame->delta));
        frame->raw_b = (uint16_t)((int32_t)pulse.zero_b + snapshot.sum[0][0] / 4);
        frame->raw_c = (uint16_t)((int32_t)pulse.zero_c + snapshot.sum[0][1] / 4);
        /* Four 20 kHz PWM samples, 1200 timer counts, 12 V bus. */
        frame->voltage_alpha_mv = snapshot.voltage_sum[0] * 5 / 6;
        frame->voltage_beta_mv = snapshot.voltage_sum[1] * 94593 >> 16;
        return true;
    }
    frame->sample_us = CURRENT_HFI_SAMPLE_US;
    int32_t current_sum[2] = {0};
    for (unsigned i = 0; i < CURRENT_PULSE_VECTORS; ++i) {
        current_sum[0] += snapshot.sum[i][0];
        current_sum[1] += snapshot.sum[i][1];
    }
    for (unsigned i = 0; i < CURRENT_PULSE_VECTORS; ++i) {
        /* Keep the observer's Q3 mean-count units independent of carrier frequency. */
        frame->delta[i][0] = (int16_t)response_divide(snapshot.sum[i][0] * 8, 38, 6899);
        frame->delta[i][1] = (int16_t)response_divide(snapshot.sum[i][1] * 8, 38, 6899);
    }
    for (unsigned i = 0; i < CURRENT_PULSE_VECTORS; i += 2) {
        frame->delta[i][0] += (int16_t)response_divide(snapshot.edge[0], 37, 7085);
        frame->delta[i][1] += (int16_t)response_divide(snapshot.edge[1], 37, 7085);
    }
    frame->raw_b = (uint16_t)((int32_t)pulse.zero_b + response_divide(current_sum[0], 152, 1725));
    frame->raw_c = (uint16_t)((int32_t)pulse.zero_c + response_divide(current_sum[1], 152, 1725));
    /* Four base-voltage snapshots in Q8 timer counts, converted at 12 V. */
    frame->voltage_alpha_mv = snapshot.voltage_sum[0] * 2133 >> 17;
    frame->voltage_beta_mv = snapshot.voltage_sum[1] * 3695 >> 17;
    return true;
}

void CurrentAdc_CancelPulse(void) {
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (pulse.active) pulse_finish(2);
    __set_PRIMASK(mask);
}

bool CurrentAdc_PulseActive(void) { return pulse.active; }
uint16_t CurrentAdc_PulseFault(void) { return pulse.fault; }
const CurrentPulseResponse *CurrentAdc_PulseResponses(void) { return measurement.response; }

static bool pulse_overcurrent(int32_t counts, int32_t limit) {
    return counts < -limit || counts > limit;
}

static inline __attribute__((always_inline)) void publish_hfi_frame(void) {
    HfiAccumulation *next = measurement.hfi.writing == &measurement.hfi.accumulation[0]
        ? &measurement.hfi.accumulation[1] : &measurement.hfi.accumulation[0];
    measurement.hfi.writing = next;
    volatile uint32_t *words = next->words;
    words[0] = 0;
    words[1] = 0;
    words[2] = 0;
    words[3] = 0;
    words[4] = 0;
    words[5] = 0;
    words[6] = 0;
    words[7] = 0;
    words[8] = 0;
    words[9] = 0;
    words[10] = 0;
    __DMB();
    ++measurement.hfi.sequence;
}

__attribute__((noinline)) static void rotating_sample(int32_t b, int32_t c) {
    /* The response belongs to the duty written at the preceding ADC event. */
    unsigned stimulus = ((pulse.vector & 1u) << 1) | (pulse.vector >> 1);
    if (pulse.wait != 0) {
        if (--pulse.wait == 0) pulse_duty(0);
        return;
    }
    if (pulse.continuous) {
        HfiAccumulation *accumulation = measurement.hfi.writing;
        if (pulse.point >= HFI_SETTLE_CYCLES) {
            accumulation->sum[stimulus][0] += b;
            accumulation->sum[stimulus][1] += c;
            if (stimulus == 0 && (pulse.point & 7u) == 0) {
                const uint16_t *base = pulse.duty[CURRENT_PULSE_VECTORS];
                accumulation->voltage_sum[0] += 2 * (int32_t)base[0] - base[1] - base[2];
                accumulation->voltage_sum[1] += (int32_t)base[1] - base[2];
            }
            /* Compare the first and last retained cycles to remove torque-current drift. */
            if (pulse.point == HFI_SETTLE_CYCLES) {
                accumulation->edge[0] -= (int16_t)b;
                accumulation->edge[1] -= (int16_t)c;
            } else if (pulse.point == CURRENT_HFI_CONTROL_CYCLES - 1) {
                accumulation->edge[0] += (int16_t)b;
                accumulation->edge[1] += (int16_t)c;
            }
        }
    } else {
        CurrentPulseResponse *result = &measurement.response[stimulus];
        result->sum[0][0] += b;
        result->sum[0][1] += c;
        result->square[0][0] += (uint32_t)(b * b);
        result->square[0][1] += (uint32_t)(c * c);
        ++result->repeats;
    }
    pulse.vector = (pulse.vector + 1u) & 3u;
    uint8_t frame_cycles = pulse.continuous ? CURRENT_HFI_CONTROL_CYCLES : CURRENT_HFI_CYCLES;
    if (pulse.vector == 0 && ++pulse.point == frame_cycles) {
        pulse.point = 0;
        if (pulse.continuous) {
            publish_hfi_frame();
        } else if (measurement.response[3].repeats == pulse.repeats * CURRENT_HFI_CYCLES) {
            pulse_finish(0);
            return;
        }
    }
    pulse_duty(((pulse.vector & 1u) << 1) | (pulse.vector >> 1));
}

__attribute__((noinline)) static void pulse_sample(uint16_t raw_b, uint16_t raw_c) {
    int32_t b = (int32_t)raw_b - pulse.zero_b;
    int32_t c = (int32_t)raw_c - pulse.zero_c;
    int32_t limit = pulse.continuous ? hfi_max_current_counts : PULSE_MAX_CURRENT_COUNTS;
    if (pulse_overcurrent(b, limit) || pulse_overcurrent(c, limit) || pulse_overcurrent(b + c, limit)) {
        pulse_finish(1);
        return;
    }
    if (pulse.rotating) {
        rotating_sample(b, c);
        return;
    }
    if (pulse.wait != 0) {
        if (--pulse.wait == 0) {
            pulse.base_b = raw_b;
            pulse.base_c = raw_c;
            pulse.point = 0;
            pulse_duty(pulse.vector);
        }
        return;
    }
    CurrentPulseResponse *result = &measurement.response[pulse.vector];
    b = (int32_t)raw_b - pulse.base_b;
    c = (int32_t)raw_c - pulse.base_c;
    result->sum[pulse.point][0] += b;
    result->sum[pulse.point][1] += c;
    result->square[pulse.point][0] += (uint32_t)(b * b);
    result->square[pulse.point][1] += (uint32_t)(c * c);
    if (++pulse.point == pulse.periods) {
        pulse_duty(CURRENT_PULSE_VECTORS);
    } else if (pulse.point > pulse.periods) {
        pulse.vector = (pulse.vector + 1u) & 3u;
        if (++result->repeats == pulse.repeats && pulse.vector == 0) {
            pulse_finish(0);
        } else {
            pulse.wait = PULSE_GAP;
        }
    }
}

void CurrentAdc_Init(void) {
    __SYSCTRL_GPIOB_CLK_ENABLE();
    __SYSCTRL_ADC_CLK_ENABLE();

    PB04_ANALOG_ENABLE();
    PB05_ANALOG_ENABLE();

    ADC_InitTypeDef ADC_InitStructure = {0};

    ADC_InitStructure.ADC_ClkDiv = ADC_Clk_Div1;
    ADC_InitStructure.ADC_ConvertMode = ADC_ConvertMode_Once;
    ADC_InitStructure.ADC_IN0.ADC_InputChannel = ADC_InputCH11;
    ADC_InitStructure.ADC_IN0.ADC_SampTime = ADC_SampTime18Clk;
    ADC_InitStructure.ADC_IN1.ADC_InputChannel = ADC_InputCH12;
    ADC_InitStructure.ADC_IN1.ADC_SampTime = ADC_SampTime18Clk;
    ADC_InitStructure.ADC_SQREns = ADC_SqrEns0to1;

    ADC_Init(&ADC_InitStructure);

    ADC_ClearITPendingAll();
    ADC_ITConfig(ADC_IT_EOS, ENABLE);
    NVIC_EnableIRQ(ADC_IRQn);

    ADC_ExtTrigCfg(ADC_TRIG_ATIMTRGO2, ENABLE);
    ADC_Enable();
}

uint32_t CurrentAdc_Next(uint16_t *raw_b, uint16_t *raw_c, uint32_t *time_us) {
    uint32_t count = consumed_sequence;
    CurrentSample snapshot;
    do {
        uint32_t available = sequence - count;
        if (available > SAMPLE_QUEUE_SIZE) count = sequence - SAMPLE_QUEUE_SIZE;
        if (available != 0) ++count;
        __DMB();
        snapshot = samples[count & (SAMPLE_QUEUE_SIZE - 1u)];
        __DMB();
        /* Retry only if the producer wrapped around the slot during the copy. */
    } while (sequence - count >= SAMPLE_QUEUE_SIZE);
    consumed_sequence = count;
    *raw_b = snapshot.b;
    *raw_c = snapshot.c;
    *time_us = snapshot.time_us;
    return count;
}

__attribute__((section(".RamFunc"), noinline)) static void continuous_sample(int32_t b, int32_t c) {
    int32_t limit = hfi_max_current_counts;
    if (pulse_overcurrent(b, limit) || pulse_overcurrent(c, limit) || pulse_overcurrent(b + c, limit)) {
        pulse_finish(1);
        return;
    }
    if (pulse.wait != 0) {
        if (--pulse.wait == 0) pulse_duty(0);
        return;
    }
    unsigned vector = pulse.vector;
    unsigned stimulus = ((vector & 1u) << 1) | (vector >> 1);
    unsigned point = pulse.point;
    HfiAccumulation *accumulation = measurement.hfi.writing;
    if (point >= HFI_SETTLE_CYCLES) {
        accumulation->sum[stimulus][0] += b;
        accumulation->sum[stimulus][1] += c;
        if (stimulus == 0 && (point & 7u) == 0) {
            const uint16_t *base = pulse.duty[CURRENT_PULSE_VECTORS];
            accumulation->voltage_sum[0] += 2 * (int32_t)base[0] - base[1] - base[2];
            accumulation->voltage_sum[1] += (int32_t)base[1] - base[2];
        }
        if (point == HFI_SETTLE_CYCLES) {
            accumulation->edge[0] -= (int16_t)b;
            accumulation->edge[1] -= (int16_t)c;
        } else if (point == CURRENT_HFI_CONTROL_CYCLES - 1) {
            accumulation->edge[0] += (int16_t)b;
            accumulation->edge[1] += (int16_t)c;
        }
    }
    vector = (vector + 1u) & 3u;
    pulse.vector = (uint8_t)vector;
    if (vector == 0) {
        if (++pulse.point == CURRENT_HFI_CONTROL_CYCLES) {
            pulse.point = 0;
            publish_hfi_frame();
        }
    }
    pulse_duty(((vector & 1u) << 1) | (vector >> 1));
}

__attribute__((noinline)) static void smo_sample(int32_t b, int32_t c) {
    int32_t limit = hfi_max_current_counts;
    if (pulse_overcurrent(b, limit) || pulse_overcurrent(c, limit) ||
        pulse_overcurrent(b + c, limit)) {
        pulse_finish(1);
        return;
    }
    HfiAccumulation *accumulation = measurement.hfi.writing;
    accumulation->sum[0][0] += b;
    accumulation->sum[0][1] += c;
    const uint16_t *base = pulse.duty[CURRENT_PULSE_VECTORS];
    accumulation->voltage_sum[0] += 2 * (int32_t)base[0] - base[1] - base[2];
    accumulation->voltage_sum[1] += (int32_t)base[1] - base[2];
    if (++pulse.point == 4) {
        pulse.point = 0;
        publish_hfi_frame();
    }
}

__attribute__((section(".RamFunc"))) void ADC_IRQHandler(void) {
    if (CW_ADC->ISR & ADC_ISR_EOS_Msk) {
        if (pulse.active) {
            uint16_t raw_b = (uint16_t)CW_ADC->RESULT0;
            uint16_t raw_c = (uint16_t)CW_ADC->RESULT1;
            if (pulse.fast_smo) smo_sample((int32_t)raw_b - pulse.zero_b,
                                          (int32_t)raw_c - pulse.zero_c);
            else if (pulse.continuous) continuous_sample((int32_t)raw_b - pulse.zero_b,
                                                         (int32_t)raw_c - pulse.zero_c);
            else pulse_sample(raw_b, raw_c);
        }
        /* Keep the foreground snapshot at 1 kHz with either PWM timing. */
        if (++pwm_periods == control_divider) {
            pwm_periods = 0;
            CurrentSample *sample = &samples[(sequence + 1u) & (SAMPLE_QUEUE_SIZE - 1u)];
            sample->b = (uint16_t)CW_ADC->RESULT0;
            sample->c = (uint16_t)CW_ADC->RESULT1;
            sample->time_us = BoardTime_NowUs();
            __DMB();
            ++sequence;
        }
    }
    CW_ADC->ICR = 0u;
}
