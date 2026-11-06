#include "hfi.h"
#include "foc.h"
#include "motion.h"

enum {
    TURN = 16384,
    HALF = TURN / 2,
    Q8 = 256,
    LOCK_FRAMES = 96,
    VELOCITY_GAIN = 2,
    MAX_VELOCITY_Q8 = MOTOR_HFI_START_SPEED_MRAD_S * 800 / 107,
};

static int32_t abs32(int32_t value) { return value < 0 ? -value : value; }

static int32_t wrap(int32_t value) {
    return (int32_t)((uint32_t)(value + HALF) & (TURN - 1u)) - HALF;
}

bool Hfi_Update(HfiObserver *observer, int16_t delta[4][2], bool locking) {
    int32_t ab = delta[0][0] - delta[1][0];
    int32_t ac = delta[0][1] - delta[1][1];
    int32_t bb = delta[2][0] - delta[3][0];
    int32_t bc = delta[2][1] - delta[3][1];
    /* Convert the symmetric alpha/beta response to Q8 ADC counts. */
    int32_t x = 8 * (ab + ac) - 37 * (bc - bb) / 8;
    int32_t y = 8 * (bb + bc) + 37 * (ac - ab) / 8;
    /* Undo the 25 kHz response bias and map the measured ellipse to rotor angle. */
    x += 44;
    y += 87;
    int32_t cx = (-14838 * x + 3786 * y) / 1024;
    int32_t cy = (-5050 * x - 16292 * y) / 1024;
    observer->x += locking ? (cx - observer->x) / 8 : (cx - observer->x) / 4;
    observer->y += locking ? (cy - observer->y) / 8 : (cy - observer->y) / 4;
    if (observer->frames < LOCK_FRAMES) ++observer->frames;
    bool visible = abs32(observer->x) + abs32(observer->y) >= 500;
    if (visible) observer->weak_frames = 0;
    else if (observer->weak_frames < 24) ++observer->weak_frames;

    int32_t measured = FOC_Atan2(observer->y, observer->x);
    if (locking) {
        /* DC alignment resolves the pi ambiguity. Choose the branch near zero. */
        observer->angle_q8 = wrap(measured) * (Q8 / 2);
        return observer->frames >= LOCK_FRAMES && visible;
    }
    if (!observer->angle_override) observer->angle_q8 += observer->velocity_q8;
    if (visible) {
        int32_t error = wrap(measured - 2 * (observer->angle_q8 / Q8)) / 2;
        observer->angle_q8 += 64 * error;
        observer->velocity_q8 += VELOCITY_GAIN * error;
        if (observer->velocity_q8 > MAX_VELOCITY_Q8) observer->velocity_q8 = MAX_VELOCITY_Q8;
        if (observer->velocity_q8 < -MAX_VELOCITY_Q8) observer->velocity_q8 = -MAX_VELOCITY_Q8;
    }
    observer->angle_q8 = (int32_t)((uint32_t)observer->angle_q8 & (TURN * Q8 - 1u));
    return observer->weak_frames < 24;
}

void Hfi_SetAngle(HfiObserver *observer, uint16_t phase) {
    observer->angle_q8 = (int32_t)(phase & (TURN - 1u)) * Q8;
    observer->angle_override = true;
}

void Hfi_ReleaseAngle(HfiObserver *observer, uint16_t phase, int32_t speed_q8) {
    observer->angle_q8 = (int32_t)(phase & (TURN - 1u)) * Q8;
    observer->velocity_q8 = speed_q8;
    observer->angle_override = false;
}

uint16_t Hfi_Angle(const HfiObserver *observer) {
    int32_t advance = observer->velocity_q8;
    return (uint16_t)((uint32_t)((observer->angle_q8 + advance) / Q8) & (TURN - 1u));
}

int32_t Hfi_Speed(const HfiObserver *observer) {
    /* Q8 electrical counts per 1.6 ms to mechanical mrad/s for seven pole pairs. */
    return observer->velocity_q8 * 107 / 800;
}
