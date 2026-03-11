#pragma once
#include <Arduino.h>

static constexpr uint8_t encoderA = 9;
static constexpr uint8_t encoderB = 11;
static constexpr uint8_t encoderI = 8;

void          pendulumEncoder_init();
float         get_pendulum_angle_rad();
bool          pendulumEncoder_isZeroed();
void          pendulumEncoder_forceZero();
unsigned long pendulumEncoder_getInvalidCount(); // non-zero = noise/wiring problem