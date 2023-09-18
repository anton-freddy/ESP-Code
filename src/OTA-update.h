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

#define LOCAL_NETWORK_MODE
//#define AP_MODE

#ifdef AP_MODE
const char *ssid = "Erasinator-3000";
const char *password = "Doofenshmirtz";
#endif

#ifdef LOCAL_NETWORK_MODE
const char *ssid = "ANTONS-ZENBOOK";
const char *password = "password";
#endif

AsyncWebServer server(80);
String HTTP_SEND;
void OTA_setup(void)
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

  server.begin();
  Serial.println("HTTP server started");
}

void OTA_loop(void)
{
}