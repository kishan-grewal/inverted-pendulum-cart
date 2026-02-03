#include <Arduino.h>
#include "drive.cpp"

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
  motor_setup();
}

void loop() {
  // put your main code here, to run repeatedly:
  set_motor_speed(1000);
  delay(2000);

  set_motor_speed(0);
  delay(500);

  set_motor_speed(-1000);
  delay(2000);

  set_motor_speed(0);
  delay(1000);
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}