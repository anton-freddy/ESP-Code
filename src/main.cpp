#include <Arduino.h>
#include <Esp.h>
#include <SpeedyStepper.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>



#define SERVO_PIN 8
#define CHANNEL 0
#define PWM_FREQ 50
#define PWM_RESOLUTION 16
#define MIN_DUTY 1000
#define MAX_DUTY 2000

const char* ssid = "Eraseinator_3000";
const char* password = "NotPerry";
SpeedyStepper S1, S2;

WebServer server(80);

//LED ints
int xCord = 0;
int yCord = 0;

void handleRoot() {
  String html = "<html><head>";
html += "<style>";
  html += "body { background-color: #F7F3E3; font-family: Arial, sans-serif; margin: 0; padding: 0; }";
  html += "header { background-color: #28536B; color: #fff; padding: 20px; text-align: center; }";
  html += "nav { background-color: #AF9164; padding: 10px; text-align: center; }";
  html += "nav a { color: #F7F3E3; padding: 10px; text-decoration: none; }";
  html += "main { text-align: center; color: #0B0500; padding: 20px; }";
  html += "footer { background-color: #28536B; color: #fff; padding: 20px; text-align: center; }";
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
  html += "<h2>Current coordinates </h2>";
  html += "<p>X: " + String(xCord) + "</p>";
  html += "<p>Y: " + String(yCord) + "</p>";
 html += "</main>";
  html += "<footer><p>&copy; 2023 Doofinschmirtz. All rights reserved.</p></footer>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}


void handleUpdate() {
  if (server.method() == HTTP_POST) {
    xCord = server.arg("value1").toInt();
    yCord = server.arg("value2").toInt();
    server.send(200, "text/plain", "Values updated");
    server.sendHeader("Refresh", "1; url=/");
  } else {
    server.send(400, "text/plain", "Invalid request");
  }
}



void setup() {
  Serial.begin(115200);

   pinMode(18, OUTPUT);
   pinMode(16, OUTPUT);
  // while (1){
  //   digitalWrite(14, HIGH);
  //   digitalWrite(13, HIGH);
  // }
  
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
  server.handleClient();

  if(S1.processMovement()){
    S1.setupMoveInMillimeters(10000);
  }
  if(S2.processMovement()){
    S2.setupMoveInMillimeters(10000);
  }



  // for LED printing
  if (xCord == 1){
    digitalWrite(18, HIGH);
  }else {
    digitalWrite(18, LOW);
  }
   if (yCord == 1){
    digitalWrite(16, HIGH);
  }else {
    digitalWrite(16, LOW);
  }
   

}