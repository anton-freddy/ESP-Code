#include <WiFi.h>
#include <WebServer.h>
// #include <WiFiClient.h>
// #include <WiFiAP.h>

const char* ssid = "Doofenschmirtz-Inc";
const char* password = "password";
//const uint16_t host = 192168001003;   //192.168.1.3
IPAddress host(192,168,001,3);
const uint16_t port = 8080;

WiFiClient client;

//WebServer server(host, port);


void network_setup() {

  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("connected");

  


  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print("\nConnecting to WiFi");
    delay(300);
    Serial.print(".");
    delay(300);
    Serial.print(".");
    delay(300);
    Serial.print(".");
    delay(300);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  //WiFiServer client;
  // Establish connection with server
  Serial.print("Connecting to server...");
  while (!client.connect(host, port)) {

    delay(1000);
    Serial.print(".");
  }
  Serial.println("connected");
  Serial.printf("Connected to server on port: %4d, IP: %4d", port, host.toString());
}
