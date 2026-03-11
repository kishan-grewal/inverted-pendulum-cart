#include "pendulum_encoder.h"

// ── Private state ─────────────────────────────────────────────────────────────
static volatile long     s_count     = 0;
static volatile bool     s_zeroed    = false;
static volatile unsigned long s_invalid = 0;  // counts physically impossible transitions
static volatile uint8_t  s_lastState = 0;

// ── Helpers ───────────────────────────────────────────────────────────────────
static inline uint8_t readEncoderState() {
    return (digitalRead(encoderA) << 1) | digitalRead(encoderB);
}

// ── ISR forward declarations ──────────────────────────────────────────────────
static void isr_quadrature();
static void isr_index();

// ── Init ──────────────────────────────────────────────────────────────────────
void pendulumEncoder_init() {
    pinMode(encoderA, INPUT_PULLUP);
    pinMode(encoderB, INPUT_PULLUP);
    pinMode(encoderI, INPUT_PULLUP);

    s_lastState = readEncoderState();

    attachInterrupt(digitalPinToInterrupt(encoderA), isr_quadrature, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderB), isr_quadrature, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderI), isr_index, RISING);
}

// ── Public API ────────────────────────────────────────────────────────────────
float get_pendulum_angle_rad() {
    noInterrupts();
    long count = s_count;
    interrupts();

    // Raw unwrapped radians — LQR needs continuous angle.
    static constexpr float COUNTS_TO_RAD = (2.0f * M_PI) / (4.0f * 1000.0f);
    return count * COUNTS_TO_RAD;
}

bool pendulumEncoder_isZeroed() {
    return s_zeroed;
}

void pendulumEncoder_forceZero() {
    noInterrupts();
    s_count  = 0;
    s_zeroed = true;
    interrupts();
}

unsigned long pendulumEncoder_getInvalidCount() {
    noInterrupts();
    unsigned long val = s_invalid;
    interrupts();
    return val;
}

// ── Quadrature ISR — transition table ────────────────────────────────────────
// CW  (+): 00→01, 01→11, 11→10, 10→00
// CCW (-): 00→10, 10→11, 11→01, 01→00
static void isr_quadrature() {
    uint8_t currentState = readEncoderState();
    uint8_t transition   = (s_lastState << 2) | currentState;

    switch (transition) {
        case 0b0001: case 0b0111: case 0b1110: case 0b1000:
            s_count++;
            break;
        case 0b0010: case 0b0100: case 0b1011: case 0b1101:
            s_count--;
            break;
        case 0b0000: case 0b0101: case 0b1010: case 0b1111:
            break;
        default:
            s_invalid++;
            break;
    }

    s_lastState = currentState;
}

static void isr_index() {
    s_count  = 0;
    s_zeroed = true;
}
