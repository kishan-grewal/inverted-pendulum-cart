// THIS IS AN INO FILE NOT A CPP FILE

// This example shows how to control the Motoron Motor Controller
// using I2C if you want your system to just keep working,
// ignoring or automatically recovering from errors as much as
// possible.
//
// The motors will stop but automatically recover if:
// - Motor power (VIN) is interrupted
// - A motor fault occurs
// - The Motoron experiences a reset
// - A command timeout occurs

#include <Motoron.h>
#include <Wire.h>


MotoronI2C mc1(16);
MotoronI2C mc2(17);

uint16_t lastTimeMotorsInit;

void motorsInit()
{
  mc1.clearResetFlag();
  mc2.clearResetFlag();

  // By default, the Motoron is configured to stop the motors if
  // it does not get a motor control command for 1500 ms.  You
  // can uncomment a line below to adjust this time or disable
  // the timeout feature.
  // mc.setCommandTimeoutMilliseconds(1000);
  // mc.disableCommandTimeout();

  // Configure motor 1
  mc1.setMaxAcceleration(1, 70);
  mc1.setMaxDeceleration(1, 150);
  mc1.clearMotorFaultUnconditional();

  // Configure motor 2
  mc2.setMaxAcceleration(1, 70);
  mc2.setMaxDeceleration(1, 150);
  mc2.clearMotorFaultUnconditional();

  lastTimeMotorsInit = millis();
}

void setup()
{
  Wire1.begin();
  
  // Set the I2C bus for both motor controllers
  mc1.setBus(&Wire1);
  mc2.setBus(&Wire1);
  
  mc1.reinitialize();
  mc2.reinitialize();
  motorsInit();
}

void loop()
{
  if (millis() & 2048)
  {
    mc1.setSpeed(1, 800);
    mc2.setSpeed(1, 800);
  }
  else
  {
    mc1.setSpeed(1, -800);
    mc2.setSpeed(1, -800);
  }

  // Every 2 seconds, run motorsInit to restart the motors
  // in case anything has caused them to shut down.
  if ((uint16_t)(millis() - lastTimeMotorsInit) > 2000)
  {
    motorsInit();
  }
}
