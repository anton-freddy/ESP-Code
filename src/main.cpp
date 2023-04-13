#include <Arduino.h>
#include <Esp.h>
#include <SpeedyStepperCustom.h>
#include <EasyRobot.h>
#include <ESP32S3_PINS.h>

EasyRobot ROOMBA(L_Stepper_STEP_PIN, L_Stepper_DIR_PIN, L_Stepper_ENABLE_PIN, R_Stepper_STEP_PIN, R_Stepper_DIR_PIN, R_Stepper_ENABLE_PIN);

void setup() {
  ROOMBA.begin(KMH, 19.1525439, 1.5, 1000);
  ROOMBA.moveTo(0, 400);
  
  // ROOMBA.leftMotor.setupMoveInMillimeters(500);
  // ROOMBA.rightMotor.setupMoveInMillimeters(500);

}

// void loop() {
//   digitalWrite(SERVO_PIN, HIGH);
//   delay(10);
//   digitalWrite(SERVO_PIN, LOW);
//   delay(10);
// }


void loop(){

 if(ROOMBA.processMovement()){
    ROOMBA.moveTo(400, 0);
    while(1){
      if(ROOMBA.processMovement()){
        ROOMBA.moveTo(0, 0);
        while(1){
          if(ROOMBA.processMovement()){
            ROOMBA.moveTo(400,400);
            while(1){
              if(ROOMBA.processMovement()){
                while(1);
              }
            }
          }
        }
      }
    }
    //ROOMBA.turn_deg();
 }
  // if(ROOMBA.processMovement()){
  //   ROOMBA.leftMotor.setCurrentPositionInMillimeters(0L);
  //   ROOMBA.rightMotor.setCurrentPositionInMillimeters(0L);
  //   ROOMBA.leftMotor.setupMoveInMillimeters(500);
  //   ROOMBA.rightMotor.setupMoveInMillimeters(500);
  // }
  
  
}