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

EasyRobot ROOMBA;

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <deque>

const char* ssid = "Eraseinator_3000";
const char* password = "NotPerry";
const int MAX_LOG_SIZE = 10;

WebServer server(80);

std::deque<String> logData;
float xCord = 0;
float yCord = 0;
float xCord_loaded = 0;
float yCord_loaded = 0;

void network_setup(void);
void addLogEntry(const String& status);
String generateLogHTML();
void handleRoot();
void handleUpdate();


void network_setup(void){

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  Serial.println("WiFi AP mode started");
  Serial.print("SSID: ");
  Serial.println(ssid);
  Serial.print("Password: ");
  Serial.println(password);
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/update", handleUpdate);

  server.begin();
  Serial.println("HTTP server started");

}

void addLogEntry(const String& status) {
  // Remove the oldest entry if the log is full
  if (logData.size() >= MAX_LOG_SIZE) {
    logData.pop_front();
  }

  // Add the new status string to the log
  logData.push_back(status);
}

String generateLogHTML() {
  String html = "<div id='log'>";
  
  // Iterate through the log entries and generate HTML for each entry
  for (const auto& entry : logData) {
    html += "<p>" + entry + "</p>";
  }
  
  html += "</div>";
  return html;
}

void handleRoot() {
  String html = "<html><head>";
  html += "<style>";
  html += "body { background-color: #F7F3E3; font-family: Arial, sans-serif; margin: 0; padding: 0; }";
  html += "header { background-color: #28536B; color: #fff; padding: 20px; text-align: center; }";
  html += "nav { background-color: #AF9164; padding: 10px; text-align: center; }";
  html += "nav a { color: #F7F3E3; padding: 10px; text-decoration: none; }";
  html += "main { text-align: center; color: #0B0500; padding: 20px; }";
  html += "footer { background-color: #28536B; color: #fff; padding: 20px; text-align: center; }";
  html += "#log { text-align: left; margin-top: 20px; padding: 10px; background-color: #E5E5E5; }";
  html += "</style>";
  html += "</head><body>";
  html += "<header><h1>Doofinschmirtz Inc</h1></header>";
  html += "<main>";
  html += "<h1>Change Coordinates</h1>";
  html += "<form action='/update' method='POST'>";
  html += "<label for='value1'>X : </label>";
  html += "<input type='number' id='value1' name='value1' value='" + String(xCord) + "'><br><br>";
  html += "<label for='value2'>Y : </label>";
  html += "<input type='number' id='value2' name='value2' value='" + String(yCord) + "'><br><br>";
  html += "<input type='submit' value='Update'>";
  html += "</form>";
  html += "<br>";
  html += "<h2>Current coordinates </h2>";
  html += "<p>X: " + String(ROOMBA.getXCoordinate()) + "</p>";
  html += "<p>Y: " + String(ROOMBA.getYCoordinate()) + "</p>";
  html += "<p>A: " + String(ROOMBA.getOrientation()) + "</p>"; 
  html += "<br>";
  html += "<br>";
  html += "<h2>Current Status </h2>";
  html += "<div id='log'>" + generateLogHTML() + "</div>";
  html += "</main>";
  html += "<footer><p>&copy; 2023 Doofinschmirtz. All rights reserved.</p></footer>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}


void handleUpdate() {
  if (server.method() == HTTP_POST) {
    xCord = server.arg("value1").toFloat();
    yCord = server.arg("value2").toFloat();
  //   server.send(200, "text/plain", "Values updated");
    server.sendHeader("Refresh", "1; url=/");
    handleRoot();
  // } else {
  //   server.send(400, "text/plain", "Invalid request");
  // }
  }
}

Pwm pwm = Pwm();

int previousMillis = 0;
int currentMillis = 0;
bool SERVO_CLOCKWISE = true;
int SERVO_pos = 0;

void core0_task(void *pvParameters);

void setup_task1(void);
void loop_task1(void);


void core0_task(void *pvParameters)
{
  for(;;){
      loop_task1();
  }
}

void setup_task1(void)
{
  network_setup();
}





//LED ints




void setup()
{
  set_IO_pins_low();
  ROOMBA.setUpPins(L_Stepper_STEP_PIN, L_Stepper_DIR_PIN, L_Stepper_ENABLE_PIN, R_Stepper_STEP_PIN, R_Stepper_DIR_PIN, R_Stepper_ENABLE_PIN);
  TaskHandle_t Task1;
  xTaskCreatePinnedToCore(core0_task, "Task 1", 100000, NULL, 1, &Task1, 0);
  // esp_task_wdt_delete(Task1);
  //  esp_task

  // esp_cpu_clear_watchpoint(0);
  // esp_task
  ROOMBA.begin(KMH, 19.1525439, 1.5, 1000);
  Serial.begin(115200);
  setup_task1();

}

void loop()
{
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

// Send data to server

void loop_task1(){
  server.handleClient();
  handleRoot();

  // for LED printing
  // if (xCord == 1){
  //   digitalWrite(18, HIGH);
  // }else {
  //   digitalWrite(18, LOW);
  // }
  //  if (yCord == 1){
  //   digitalWrite(16, HIGH);
  // }else {
  //   digitalWrite(16, LOW); 
  if (!(xCord == xCord_loaded && yCord == yCord_loaded)){
    xCord_loaded = xCord;
    yCord_loaded = yCord;
    ROOMBA.moveTo(xCord, yCord);
  }
  
    
  String status = "New status";  // Replace with your actual status string
  addLogEntry("X: " + (String)ROOMBA.getXCoordinate() + "  Y: " + (String)ROOMBA.getYCoordinate() + "  A: " + (String)ROOMBA.getOrientation());
  
  // Handle client requests
  //server.handleClient();
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
