#include <Arduino.h>
#include <Esp.h>
#include <SpeedyStepperCustom.h>
#include <EasyRobot.h>
#include <ESP32S3_PINS.h>
#include <pwmWrite.h>

EasyRobot ROOMBA(L_Stepper_STEP_PIN, L_Stepper_DIR_PIN, L_Stepper_ENABLE_PIN, R_Stepper_STEP_PIN, R_Stepper_DIR_PIN, R_Stepper_ENABLE_PIN);

Pwm pwm = Pwm();

int previousMillis = 0;
int currentMillis = 0;
bool SERVO_CLOCKWISE = true;
int SERVO_pos = 0;

void setup() {
  ROOMBA.begin(KMH, 19.1525439, 1.5, 1000);
  Serial.begin(115200);
  ROOMBA.moveTo(0, 500);
  for (int pos = 0; pos <= 60; pos++) {  // go from 0-180 degrees
    pwm.writeServo(SERVO_PIN, pos);        // set the servo position (degrees)
    
    delay(15);
  }
  SERVO_pos = 60;
  // ROOMBA.leftMotor.setupMoveInMillimeters(500);
  // ROOMBA.rightMotor.setupMoveInMillimeters(500);
  //ROOMBA.turn(2*PI);
}



// void loop(){

//   ROOMBA.processMovement();
  
//   currentMillis = millis();

//   if(currentMillis > previousMillis + 5){
//     if(SERVO_CLOCKWISE){
//       if(SERVO_pos <= 110){
//         previousMillis = currentMillis;
//         pwm.writeServo(SERVO_PIN, SERVO_pos);
//         SERVO_pos++;
//       }else{
//         SERVO_CLOCKWISE = false;
//       }
//     }else{
//       if(SERVO_pos >= 10){
//         previousMillis = currentMillis;
//         pwm.writeServo(SERVO_PIN, SERVO_pos);
//         SERVO_pos--;
//       }else{
//         SERVO_CLOCKWISE = true;
//       }
//     }
//   }
// }
void loop(){
  start:
  currentMillis = millis();
  if (currentMillis > previousMillis + 300){
    previousMillis = currentMillis;
    Serial.println("X: " + (String)ROOMBA.getXCoordinate() + "  Y: " + (String)ROOMBA.getYCoordinate() + "  A: " + (String)ROOMBA.getOrientation());
  }

  //Serial.println("X: " + (String)ROOMBA.getXCoordinate() + "  Y: " + (String)ROOMBA.getYCoordinate() + "  A: " + (String)ROOMBA.getOrientation());
 if(ROOMBA.processMovement()){
    ROOMBA.moveTo(500, 500);
    while(1){
      currentMillis = millis();
      if (currentMillis > previousMillis + 300){
        previousMillis = currentMillis;
        Serial.println("X: " + (String)ROOMBA.getXCoordinate() + "  Y: " + (String)ROOMBA.getYCoordinate() + "  A: " + (String)ROOMBA.getOrientation());
      }
      if(ROOMBA.processMovement()){
        ROOMBA.moveTo(500, 0);
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
                ROOMBA.moveTo(0, 500);
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

// void loop(){
//   if(ROOMBA.processMovement()){
//     delay(500);
//     ROOMBA.turn(2*PI);
//   }
// }