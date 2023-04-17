#include <Arduino.h>
#include <Esp.h>
#include <SpeedyStepperCustom.h>
#include <EasyRobot.h>
#include <ESP32S3_PINS.h>

EasyRobot ROOMBA(L_Stepper_STEP_PIN, L_Stepper_DIR_PIN, L_Stepper_ENABLE_PIN, R_Stepper_STEP_PIN, R_Stepper_DIR_PIN, R_Stepper_ENABLE_PIN);

void setup() {
  ROOMBA.begin(KMH, 19.1525439, 1.5, 1000);
  Serial.begin(115200);
  ROOMBA.moveTo(0, 1000);

  
  // ROOMBA.leftMotor.setupMoveInMillimeters(500);
  // ROOMBA.rightMotor.setupMoveInMillimeters(500);

}

int previousMillis = 0;
int currentMillis = 0;
void loop(){
  start:
  currentMillis = millis();
  if (currentMillis > previousMillis + 300){
    previousMillis = currentMillis;
    Serial.println("X: " + (String)ROOMBA.getXCoordinate() + "  Y: " + (String)ROOMBA.getYCoordinate() + "  A: " + (String)ROOMBA.getOrientation());
  }

  //Serial.println("X: " + (String)ROOMBA.getXCoordinate() + "  Y: " + (String)ROOMBA.getYCoordinate() + "  A: " + (String)ROOMBA.getOrientation());
 if(ROOMBA.processMovement()){
    ROOMBA.moveTo(1000, 0);
    while(1){
      currentMillis = millis();
      if (currentMillis > previousMillis + 300){
        previousMillis = currentMillis;
        Serial.println("X: " + (String)ROOMBA.getXCoordinate() + "  Y: " + (String)ROOMBA.getYCoordinate() + "  A: " + (String)ROOMBA.getOrientation());
      }
      if(ROOMBA.processMovement()){
        ROOMBA.moveTo(1000, 1000);
        while(1){
          currentMillis = millis();
          if (currentMillis > previousMillis + 300){
            previousMillis = currentMillis;
            Serial.println("X: " + (String)ROOMBA.getXCoordinate() + "  Y: " + (String)ROOMBA.getYCoordinate() + "  A: " + (String)ROOMBA.getOrientation());
          }
          if(ROOMBA.processMovement()){
            ROOMBA.moveTo(0,0);
            while(1){
              currentMillis = millis();
              if (currentMillis > previousMillis + 300){
                previousMillis = currentMillis;
                Serial.println("X: " + (String)ROOMBA.getXCoordinate() + "  Y: " + (String)ROOMBA.getYCoordinate() + "  A: " + (String)ROOMBA.getOrientation());
              }
              if(ROOMBA.processMovement()){
                ROOMBA.moveTo(0, 1000);
                goto start;
                
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