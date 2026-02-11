// THIS IS AN INO FILE NOT A CPP FILE

#include <Motoron.h>
#include <Wire.h>

// --- USER CONFIGURATION ---
const float GEAR_RATIO = 9.68;
const int COUNTS_PER_MOTOR_REV = 48;
const float WHEEL_DIAMETER_METERS = 0.070;
const float COUNTS_PER_WHEEL_REV = COUNTS_PER_MOTOR_REV * GEAR_RATIO;

// --- PINS ---
const int m1pinA = 2; 
const int m1pinB = 3; 

// --- STATE VARIABLES ---
volatile long encoderCount = 0;

// We move these OUT of the function so we can set them correctly in setup
volatile int lastStateA;
volatile int lastStateB;

void setup() {
  Serial.begin(9600);
  Wire1.begin(); // Required for Motoron

  // 1. PINMODE FIX: Use INPUT, not INPUT_PULLUP
  // (We rely on the encoder's strong signal or external resistors)
  pinMode(m1pinA, INPUT);
  pinMode(m1pinB, INPUT);

  // 2. LOGIC FIX: Read the initial state BEFORE enabling interrupts
  lastStateA = digitalRead(m1pinA);
  lastStateB = digitalRead(m1pinB);

  attachInterrupt(digitalPinToInterrupt(m1pinA), handleEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(m1pinB), handleEncoder, CHANGE);
  
  Serial.print("Counts per Wheel Revolution: ");
  Serial.println(COUNTS_PER_WHEEL_REV);
}

void loop() {
  static long lastCount = 0;
  
  noInterrupts();
  long currentCount = encoderCount;
  interrupts();

  if (currentCount != lastCount) {
    lastCount = currentCount;
    float revolutions = currentCount / COUNTS_PER_WHEEL_REV;
    float distanceMeters = revolutions * (PI * WHEEL_DIAMETER_METERS);

    Serial.print("Revs: ");
    Serial.print(revolutions, 3);
    Serial.print(" | Dist: ");
    Serial.print(distanceMeters, 3);
    Serial.println(" m");
  }
}

// Optimized 4X ISR
void handleEncoder() {
  int stateA = digitalRead(m1pinA);
  int stateB = digitalRead(m1pinB);

  // LOGIC FIX: We use the global volatile variables now
  if (stateA != lastStateA) {
    if (stateA != stateB) encoderCount++; else encoderCount--;
  }
  else if (stateB != lastStateB) {
    if (stateA == stateB) encoderCount++; else encoderCount--;
  }
  
  lastStateA = stateA;
  lastStateB = stateB;
}