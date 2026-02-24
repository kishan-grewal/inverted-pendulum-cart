#include <Motoron.h>
#include <Wire.h>

MotoronI2C motor_driver_front(17);
MotoronI2C motor_driver_back(16);

void motors_init()
{
  motor_driver_front.clearResetFlag();
  motor_driver_back.clearResetFlag();

  // left motor
  motor_driver_front.setMaxAcceleration(1, 70);
  motor_driver_front.setMaxDeceleration(1, 150);
  // right motor
  motor_driver_front.setMaxAcceleration(2, 70);
  motor_driver_front.setMaxDeceleration(2, 150);

  motor_driver_front.clearMotorFaultUnconditional();

  // left motor
  motor_driver_back.setMaxAcceleration(1, 70);
  motor_driver_back.setMaxDeceleration(1, 150);
  // right motor
  motor_driver_back.setMaxAcceleration(2, 70);
  motor_driver_back.setMaxDeceleration(2, 150);

  motor_driver_back.clearMotorFaultUnconditional();
}

void motor_setup()
{
  Wire1.begin();
  
  // Set the I2C bus for both motor controllers
  motor_driver_front.setBus(&Wire1);
  motor_driver_back.setBus(&Wire1);
  
  motor_driver_front.reinitialize();
  motor_driver_back.reinitialize();
  motors_init();
}

void set_motor_speed(int16_t speed)
{
  // Set all four motors to the same speed
  motor_driver_front.setSpeed(1, speed);  // front motor 1
  motor_driver_front.setSpeed(2, speed);  // front motor 2
  motor_driver_back.setSpeed(1, speed);   // back motor 1
  motor_driver_back.setSpeed(2, speed);   // back motor 2
}

void set_motor_speeds(int16_t front_left, int16_t front_right, int16_t back_left, int16_t back_right)
{
  motor_driver_front.setSpeed(1, front_left);
  motor_driver_front.setSpeed(2, front_right);
  motor_driver_back.setSpeed(1, back_left);
  motor_driver_back.setSpeed(2, back_right);
}