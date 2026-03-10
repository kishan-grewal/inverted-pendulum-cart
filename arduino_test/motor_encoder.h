#pragma once
#include <Arduino.h>


// Motor Encoder - 464.64 counts per revolution
const float GEAR_RATIO = 9.68;
const int COUNTS_PER_MOTOR_REV = 48; // 12 CPR * 4
const float WHEEL_DIAMETER_METERS = 0.070; // 70mm wheels
const float DIST_PER_COUNT = (PI * WHEEL_DIAMETER_METERS) / (COUNTS_PER_MOTOR_REV * GEAR_RATIO); // where is PI defined

class MotorEncoder{
private:
  volatile int lastStateA, lastStateB;

public:
  const int pinA, pinB;
  volatile long count = 0;

  MotorEncoder(int a, int b);
  void init();
  void update();
};

extern MotorEncoder ME1;
extern MotorEncoder ME2;
extern MotorEncoder ME3;
extern MotorEncoder ME4;

void motorEncoders_init();
void encoders_getDistance(float *d1 = nullptr, float *d2 = nullptr, float *d3 = nullptr, float *d4 = nullptr);