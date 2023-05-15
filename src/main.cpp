#include <Arduino.h>
#include <Esp.h>
#include <SpeedyStepperCustom.h>
#include <EasyRobot.h>
#include <ESP32S3_PINS.h>
#include <pwmWrite.h>
#include <Connection.h>
#include <string>
#include <iostream>
#include "esp_task_wdt.h"

EasyRobot ROOMBA(L_Stepper_STEP_PIN, L_Stepper_DIR_PIN, L_Stepper_ENABLE_PIN, R_Stepper_STEP_PIN, R_Stepper_DIR_PIN, R_Stepper_ENABLE_PIN);

Pwm pwm = Pwm();

int previousMillis = 0;
int currentMillis = 0;
bool SERVO_CLOCKWISE = true;
int SERVO_pos = 0;

void core0_task(void *pvParameters);

void loop_task1(void);
void loop_task2(void);
void setup_task1(void);

void core0_task(void *pvParameters)
{
  setup_task1();
  while (1)
  {
    loop_task1();
  }
}

void setup_task1(void)
{
  network_setup();
}

void setup()
{

  TaskHandle_t Task1;
  xTaskCreatePinnedToCore(core0_task, "Task 1", 100000, NULL, 1, &Task1, 0);
  // esp_task_wdt_delete(Task1);
  //  esp_task

  // esp_cpu_clear_watchpoint(0);
  // esp_task
  ROOMBA.begin(KMH, 19.1525439, 1.5, 1000);
  Serial.begin(115200);
  setup_task1();
  // ROOMBA.moveTo(0, 500);
  // for (int pos = 0; pos <= 60; pos++)
  // {                                 // go from 0-180 degrees
  //   pwm.writeServo(SERVO_PIN, pos); // set the servo position (degrees)

  //   delay(15);
  // }

  SERVO_pos = 60;
  // ROOMBA.leftMotor.setupMoveInMillimeters(500);
  // ROOMBA.rightMotor.setupMoveInMillimeters(500);
  // ROOMBA.turn(2*PI);
}

void loop_task1()
{

  // while(1){
  //  Check if a client has connected
  // WiFiClient client = server.available();
  // if (client.connected())
  // {
  //   Serial.println("New client connected");
  // }

  // while (client.available())
  //   {

  //     // Read the command sent by the client
  //     String command = "";

  //     // Serial.println("IN WHILE");
  //     // if (client.available())
  //     // {
  //     // Serial.println("clien available");
  //     char c = ' ';
  //     while (1)
  //     {
  //       c = client.read();
  //       if (c == '\n')
  //       {
  //         Serial.println(command);
  //         Serial.println("new line found");
  //         break;
  //       }
  //       command = +c;
  //     }
  //     client.print("OK");

  //     //}
  //     //}

  //     // if(command != ""){
  //     //   client.print("OK");
  //     //   Serial.println("Command received: " + command);

  //     // Print the command received from the client

  //     int char_size = command.length();
  //     char comm[char_size];
  //     for (int i = 0; i < command.length(); i++)
  //     {
  //       comm[i] = command.charAt(i);
  //     }

  //     if (comm[0] == 'X' && comm[6] == 'Y')
  //     {
  //       String x_str = (String)comm[1] + (String)comm[2] + (String)comm[3] + (String)comm[4];
  //       String y_str = (String)comm[7] + (String)comm[8] + (String)comm[9] + (String)comm[10];
  //       float xt = x_str.toFloat();
  //       float yt = y_str.toFloat();
  //       ROOMBA.moveTo(xt, yt);
  //     }
  //   }

  // if (!ROOMBA.processMovement())
  // {
  //   client.println("X: " + (String)ROOMBA.getXCoordinate() + "  Y: " + (String)ROOMBA.getYCoordinate() + "  A: " + (String)ROOMBA.getOrientation());
  // }

  // // Send a response to the client

  // client.print("TEST");

  // // Close the connection
  // client.stop();
  // Serial.println("Client disconnected");
  // // }else{
  // //   return;
  // // }
  // //}

///////////////////////////////////////////////////////////////////////////////////////////

// Send data to server
  client.println("Hello from Arduino!");

  // Receive response from server
  String response = "";
  while (client.available()) {
    response += (char)client.read();
  }
  if (response != "") {
    Serial.print("Response from server: ");
    Serial.println(response);
  }

  delay(1000);


}

void loop_task2()
{

  // loop_task1();
  currentMillis = millis();
  if (currentMillis > previousMillis + 300)
  {
    previousMillis = currentMillis;
    Serial.println("X: " + (String)ROOMBA.getXCoordinate() + "  Y: " + (String)ROOMBA.getYCoordinate() + "  A: " + (String)ROOMBA.getOrientation());
  }

  // Serial.println("X: " + (String)ROOMBA.getXCoordinate() + "  Y: " + (String)ROOMBA.getYCoordinate() + "  A: " + (String)ROOMBA.getOrientation());
  if (ROOMBA.processMovement())
  {
  }
}

void loop()
{
  loop_task2();
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
