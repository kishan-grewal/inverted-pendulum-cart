#include <Arduino.h>
#include "drive.h"
#include "encoder.h"

float encoder_angle = 0.0;
const float pulses_per_revolution = 1000;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    // Wait for Serial port to connect (up to 3 seconds)
  }
  
  Serial.println("Initialising Motoron controller...");
  motor_setup();
  Serial.println("Motoron initialisation complete.");

  Serial.println("Initialising Encoder...");
  encoder_setup();
  Serial.println("Encoder initialisation complete.");
}

void loop() {
  noInterrupts();
  long current_count = pulse_count;
  interrupts();

  Serial.print("Position (Pulses): ");
  Serial.println(current_count);

  encoder_angle = (current_count / pulses_per_revolution) * 360.0;

  if (encoder_angle > 360.0) {
    encoder_angle = fmod(encoder_angle, 360.0);

  } else if (encoder_angle < 0.0) {
    encoder_angle += 360.0;
  }

  Serial.println("Encoder Angle (Degrees): " + String(encoder_angle, 2));

  delay(50);

}