/*
  Rui Santos
  Complete project details
   - Arduino IDE: https://RandomNerdTutorials.com/esp32-ota-over-the-air-arduino/
   - VS Code: https://RandomNerdTutorials.com/esp32-ota-over-the-air-vs-code/

  This sketch shows a Basic example from the AsyncElegantOTA library: ESP32_Async_Demo
  https://github.com/ayushsharma82/AsyncElegantOTA
*/

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <ESPmDNS.h>
#include <ESPDash.h>

//#define LOCAL_NETWORK_MODE
#define AP_MODE

#ifdef AP_MODE
const char *ssid = "Erasinator-3000";
const char *password = "Doofenshmirtz";
#endif

#ifdef LOCAL_NETWORK_MODE
const char *ssid = "ANTONS-ZENBOOK";
const char *password = "password";
#endif

struct webservervar {
  int led_on;
  int motor_on;
};



int update_led();
void web_setup(void);
void web_loop(void);
void update_lidar_readings(int LiDAR1, int LiDAR2);

webservervar server_var;

AsyncWebServer server(80);

  
ESPDash dashboard(&server);

Card led_button(&dashboard, BUTTON_CARD, "LED");
Card motor_enable(&dashboard, BUTTON_CARD, "MOTOR STATUS");
Card card_lidar_1(&dashboard, GENERIC_CARD, "LiDAR 1 Distance", "CM");
Card card_lidar_2(&dashboard, GENERIC_CARD, "LiDAR 2 Distance","CM");
Card card_lidar_1_abs(&dashboard, GENERIC_CARD, "Object Detected");
Card card_lidar_2_abs(&dashboard, GENERIC_CARD, "Object Detected");
Card card_bump_state(&dashboard, GENERIC_CARD, "BUMP State");
Card card_charge_level(&dashboard, GENERIC_CARD, "Battery Level", "%");

String HTTP_SEND;

void web_setup(void)
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

#ifdef AP_MODE
  WiFi.softAP(ssid, password);
  Serial.println("");
  Serial.print("Network Created: ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());
  IPAddress APip(WiFi.softAPIP());

  HTTP_SEND = "Erasinator 3000 main controller - head to " + (String)APip.toString() + "/update to update the firmware";

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", HTTP_SEND); });
#endif

#ifdef LOCAL_NETWORK_MODE
WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(10);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  IPAddress APip(WiFi.localIP());
  HTTP_SEND = "Erasinator 3000 main controller - head to " + (String)APip.toString() + "/update to update the firmware";
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", HTTP_SEND); });
#endif

  if(!MDNS.begin("Eraseinator3000")) {
     Serial.println("Error starting mDNS");
}

  AsyncElegantOTA.begin(&server); // Start ElegantOTA

    /* Attach Button Callback */
  led_button.attachCallback([&](int value){
    /* Print our new button value received from dashboard */
    server_var.led_on = value;
    /* Make sure we update our button's value and send update to dashboard */
    led_button.update(value);
    dashboard.sendUpdates();
  });

  motor_enable.attachCallback([&](int value){
    /* Print our new button value received from dashboard */
    server_var.motor_on = value;
    /* Make sure we update our button's value and send update to dashboard */
    motor_enable.update(value);
    dashboard.sendUpdates();
  });

  server.begin();
  Serial.println("HTTP server started");
}

void web_loop(void)
{
  
}

void update_lidar_readings(int LiDAR1, int LiDAR2){

}

