#include <Arduino.h>
#include <Esp.h>
#include <SpeedyStepper.h>

#define SERVO_PIN 8
#define CHANNEL 0
#define PWM_FREQ 50
#define PWM_RESOLUTION 16
#define MIN_DUTY 1000
#define MAX_DUTY 2000

SpeedyStepper S1;

void setup() {
  Serial.begin(115200);
  S1.connectToPins(11,10);
  pinMode(SERVO_PIN, OUTPUT);
  S1.setAccelerationInRevolutionsPerSecondPerSecond(20);
  S1.setSpeedInRevolutionsPerSecond(20);
}

// void loop() {
//   digitalWrite(SERVO_PIN, HIGH);
//   delay(10);
//   digitalWrite(SERVO_PIN, LOW);
//   delay(10);
// }


void loop(){

  S1.moveRelativeInRevolutions(600);


}