#include "foc.h"
#include "foc_tables.h"

#define Q14 16384
#define INV_SQRT3_Q14 9459
#define HALF_SQRT3_Q14 14189

static uint16_t supply_mv = 12000;
static uint16_t supply_limit_mv = 6928;
static uint32_t normal_gain_q21 = 209540;
static uint32_t hfi_gain_q16 = 335544;

void FOC_SetBusVoltage(uint16_t bus_mv) {
    supply_mv = bus_mv;
    supply_limit_mv = (uint16_t)((uint32_t)bus_mv * 37837u >> 16);
    normal_gain_q21 = (1199u * (1u << 21) + bus_mv / 2u) / bus_mv;
    hfi_gain_q16 = (61440u * (1u << 16) + bus_mv / 2u) / bus_mv;
}

uint16_t FOC_GetBusVoltage(void) { return supply_mv; }
uint16_t FOC_GetVoltageLimit(void) { return supply_limit_mv; }

FocAngle FOC_Angle(uint16_t phase) {
    unsigned offset = phase & 4095u;
    int32_t s = sine_quarter[offset];
    int32_t c = sine_quarter[4096u - offset];
    if (phase & 4096u) {
        int32_t previous_s = s;
        s = c;
        c = -previous_s;
    }
    if (phase & 8192u) {
        s = -s;
        c = -c;
    }
    return (FocAngle){s, c};
}

uint16_t FOC_Atan2(int32_t y, int32_t x) {
    uint32_t ax = x < 0 ? -(uint32_t)x : (uint32_t)x;
    uint32_t ay = y < 0 ? -(uint32_t)y : (uint32_t)y;
    uint32_t major = ax >= ay ? ax : ay;
    uint32_t minor = ax >= ay ? ay : ax;
    if (major == 0) return 0;
    /* Normalize together to retain 15 fractional bits without a 64-bit product.
     * The Q31 reciprocal keeps the Q10 ratio error below two counts. */
    unsigned shift = (unsigned)__builtin_clz(major);
    uint32_t denominator = (major << shift) >> 16;
    uint32_t numerator = (minor << shift) >> 16;
    uint32_t reciprocal = atan_reciprocal[(denominator >> 6) - 512u];
    uint32_t ratio = (numerator * reciprocal + (1u << 20)) >> 21;
    if (ratio > 1024u) ratio = 1024u;
    int32_t angle = atan_octant[ratio];
    if (ax < ay) angle = 4096 - angle;
    if (x < 0) angle = 8192 - angle;
    if (y < 0) angle = -angle;
    return (uint16_t)((uint32_t)angle & 16383u);
}

uint32_t FOC_Sqrt(uint32_t value) {
    if (value == 0) return 0;
    /* An even normalization shift preserves the square-root scale.
     * The table starts at or above floor(sqrt(value)). One Newton step
     * then leaves only a possible one-count excess, including UINT32_MAX. */
    unsigned shift = (unsigned)__builtin_clz(value) & ~1u;
    unsigned index = (value << shift) >> 22;
    uint32_t root = sqrt_seed[index - 256u] >> (shift / 2u);
    root = (root + value / root) / 2u;
    if (root > 65535u || root * root > value) --root;
    return root;
}

uint32_t FOC_SqrtLowerBound(uint32_t value) {
    if (value == 0) return 0;
    unsigned shift = (unsigned)__builtin_clz(value) & ~1u;
    unsigned index = (value << shift) >> 22;
    /* The previous bin's upper bound minus one lies below this bin.
     * Reuse the existing table without a Newton division or extra RAM. */
    uint32_t root = index == 256u ? 32768u : sqrt_seed[index - 257u] - 1u;
    return root >> (shift / 2u);
}

Dq FOC_Park(int32_t current_a_ma, int32_t current_b_ma, FocAngle angle) {
    int32_t beta = (current_a_ma + 2 * current_b_ma) * INV_SQRT3_Q14 / Q14;
    return (Dq){
        .d = (current_a_ma * angle.cos_q14 + beta * angle.sin_q14) / Q14,
        .q = (beta * angle.cos_q14 - current_a_ma * angle.sin_q14) / Q14,
    };
}

static uint16_t compare(int32_t voltage, uint16_t bus_mv, uint16_t pwm_max) {
    if (voltage < 0) return 0;
    if (voltage > bus_mv) return pwm_max;
    if (bus_mv == 12000 && pwm_max == 61440) {
        /* 100 kHz PWM, Q8 timer counts. 61440/12000 = 128/25. */
        uint32_t quotient = (uint32_t)voltage * 5243u >> 17;
        uint32_t remainder = (uint32_t)voltage - quotient * 25u;
        return (uint16_t)(quotient * 128u + ((remainder * 128u + 12u) * 5243u >> 17));
    }
    if (bus_mv == 12000 && pwm_max <= 1199) {
        /* Exact quotient for the board's 12 V, <=1200-count PWM range.
         * Reducing first keeps the reciprocal product within 32 bits. */
        uint32_t numerator = (uint32_t)voltage * pwm_max + 6000u;
        uint32_t quotient = ((numerator >> 8) * 1399u) >> 16;
        if (quotient * 12000u > numerator) --quotient;
        if ((quotient + 1u) * 12000u <= numerator) ++quotient;
        return (uint16_t)quotient;
    }
    if (bus_mv == supply_mv && pwm_max == 61440) {
        return (uint16_t)(((uint32_t)voltage * hfi_gain_q16 + 32768u) >> 16);
    }
    if (bus_mv == supply_mv && pwm_max == 1199) {
        return (uint16_t)(((uint32_t)voltage * normal_gain_q21 + (1u << 20)) >> 21);
    }
    return (uint16_t)((voltage * pwm_max + bus_mv / 2) / bus_mv);
}

PhaseDuty FOC_Modulate(Dq voltage, FocAngle angle, uint16_t bus_mv, uint16_t pwm_max) {
    int32_t alpha = (voltage.d * angle.cos_q14 - voltage.q * angle.sin_q14) / Q14;
    int32_t beta = (voltage.d * angle.sin_q14 + voltage.q * angle.cos_q14) / Q14;
    return FOC_ModulateStationary(alpha, beta, bus_mv, pwm_max);
}

PhaseDuty FOC_ModulateStationary(int32_t alpha, int32_t beta, uint16_t bus_mv, uint16_t pwm_max) {
    int32_t a = alpha;
    int32_t b = -alpha / 2 + beta * HALF_SQRT3_Q14 / Q14;
    int32_t c = -alpha / 2 - beta * HALF_SQRT3_Q14 / Q14;
    int32_t min = a < b ? a : b;
    int32_t max = a > b ? a : b;
    if (c < min) min = c;
    if (c > max) max = c;
    int32_t common = (bus_mv - min - max) / 2;
    return (PhaseDuty){
        compare(a + common, bus_mv, pwm_max),
        compare(b + common, bus_mv, pwm_max),
        compare(c + common, bus_mv, pwm_max),
    };
}
