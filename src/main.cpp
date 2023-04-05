#include <Arduino.h>
#include <Esp.h>
#include <SpeedyStepper.h>

#define SERVO_PIN 8
#define CHANNEL 0
#define PWM_FREQ 50
#define PWM_RESOLUTION 16
#define MIN_DUTY 1000
#define MAX_DUTY 2000

SpeedyStepper S1, S2;

void setup() {
  Serial.begin(115200);
  S1.connectToPins(13,14);
  S2.connectToPins(11,12);
  //S2.setStepsPerRevolution(400);
  S2.setStepsPerMillimeter(20.54647908);
  //S1.setStepsPerRevolution(400);
  S1.setStepsPerMillimeter(20.54647908);
  
  S1.setAccelerationInMillimetersPerSecondPerSecond(300);
  S1.setSpeedInMillimetersPerSecond(1500);

  
  S2.setAccelerationInMillimetersPerSecondPerSecond(300);
  S2.setSpeedInMillimetersPerSecond(1500);

  S1.setupMoveInMillimeters(10000);
  S2.setupMoveInMillimeters(10000);
}


void loop(){

  if(S1.processMovement()){
    S1.setupMoveInMillimeters(10000);
  }
  if(S2.processMovement()){
    S2.setupMoveInMillimeters(10000);
  }

}