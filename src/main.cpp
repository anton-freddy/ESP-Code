#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

int counter = 0;

// ESPtesting
String testingdata = "";
const char* ssid = "ESPtesting";
const char* password = "ESPtesting";
IPAddress staticIP(192, 168, 137, 100); // Replace with your desired static IP address
IPAddress gateway(192, 168, 137, 1);    // Replace with your network gateway IP address
IPAddress subnet(255, 255, 255, 0);   // Replace with your network subnet mask

WebServer server(80);
WebSocketsServer webSocketServer(81);

String logData = "";
String xyPlaneData = "";

void handleRoot() {
    String html = "<!DOCTYPE html>";
    html += "<html>";
    html += "<head>";
    html += "<title>Doofinschmirtz Inc.</title>";
    html += "<script src=\"https://ajax.googleapis.com/ajax/libs/jquery/3.6.0/jquery.min.js\"></script>";
    html += "<script>";
    html += "function sendData() {";
    html += "  var xValue = document.getElementById('inputX').value;";
    html += "  var yValue = document.getElementById('inputY').value;";
    html += "  var data = { x: xValue, y: yValue };";
    html += "  $.ajax({";
    html += "    url: '/api',";
    html += "    type: 'POST',";
    html += "    data: data,";
    html += "    success: function(response) {";
    html += "      document.getElementById('outputValue').innerText = response;";
    html += "    }";
    html += "  });";
    html += "}";
    html += "</script>";

   html += "<style>";
html += "/* add your CSS styles here */";
html += "body {";
html += "    background-color: #F7F3E3;";
html += "    font-family: Arial, sans-serif;";
html += "    margin: 0;";
html += "    padding: 0;";
html += "}";
html += "";
html += "header {";
html += "    background-color: #28536B;";
html += "    color: #fff;";
html += "    padding: 20px;";
html += "    text-align: center;";
html += "}";
html += "";
html += "nav {";
html += "    background-color: #AF9164;";
html += "    padding: 10px;";
html += "    text-align: center;";
html += "}";
html += "";
html += "nav a {";
html += "    color: #F7F3E3;";
html += "    padding: 10px;";
html += "    text-decoration: none;";
html += "}";
html += "";
html += "main {";
html += "    text-align: center;";
html += "    color: #0B0500;";
html += "    padding: 20px;";
html += "}";
html += "";
html += "footer {";
html += "    background-color: #28536B;";
html += "    color: #fff;";
html += "    padding: 20px;";
html += "    text-align: center;";
html += "}";
html += "";
html += "#logBox {";
html += "    height: 200px;";
html += "    overflow-y: scroll;";
html += "    border: 1px solid #ccc;";
html += "    background-color: #ffffff;";
html += "    padding: 10px;";
html += "}";
html += "";
html += ".x-axis {";
html += "     position: absolute;";
html += "     width: 100%;";
html += "     height: 2px;";
html += "     background-color: #000;";
html += "     left: 0;";
html += "     top: 50%;";
html += "     transform: translateY(-50%);";
html += "   }";
html += " .y-axis {";
html += "    position: absolute;";
html += "    width: 2px;";
html += "    height: 100%;";
html += "     background-color: #000;";
html += "     left: 50%;";
html += "     top: 0;";
html += "     transform: translateX(-50%);";
html += "    }";

html += "  #xyPlane {";
html += "   position: relative;";
html += "   width: 800px;"; // Make the plane 800x800 pixels
html += "   height: 800px;"; // Make the plane 800x800 pixels
html += "   background-color: #e0e0e0;"; // Set the background color
html += "  }";
html += " #xyPlaneContainer {";
html += "    display: flex;";
html += "   flex-direction: column-reverse;"; // Place the xyPlane below the logBox
html += "   align-items: center;";
html += "   height: calc(100vh - 200px);"; // Adjust the height to fit the remaining space
html += "   overflow: hidden;"; // Hide any overflow if it occurs
html += " }";
html += "</style>";
    html += "</head>";

    html += "<body>";
    html += "<header>";
    html += "<h1>Doofinschmirtz Inc</h1>";
    html += "</header>";
    html += "<nav>";
    html += "<a href=\"/\">Home</a>";
    html += "</nav>";

    html += "<main>";
    html += "<input type=\"text\" id=\"inputX\" placeholder=\"X Coordinate\">";
    html += "<br><br>";
    html += "<input type=\"text\" id=\"inputY\" placeholder=\"Y Coordinate\">";
    html += "<br><br>";
    html += "<button onclick=\"sendData()\">Send</button>";
    html += "<br><br>";
    html += "<div id=\"outputValue\"></div>";
    html += "<div id=\"logBox\">" + logData + "</div>";
    html += "<br><br>";
    html += "<div id='xyPlaneContainer' style='display: flex; justify-content: center; align-items: center; '>";
    html += "<div id='xyPlane' style='position: relative; width: 800px; height: 800px; background-color: #e0e0e0;'>";
    html += "<div class='x-axis'></div>";
    html += "<div class='y-axis'></div>";
    html += "</div>";
    html += "</div>";
    html += "<br><br>";
    html += "</main>";

    html += "<footer>";
    html += "<p>&copy; 2023 Doofinschmirtz. All rights reserved.</p>";
    html += "</footer>";

    html += "<script>";
    html += "var webSocket = new WebSocket('ws://' + window.location.hostname + ':81/');";
    html += "var xyPlane = document.getElementById('xyPlane');";

    html += "webSocket.onmessage = function(event) {";
    html += "  var logBox = document.getElementById('logBox');";
    html += "  logBox.innerHTML += '<div>' + event.data + '</div>';";
    html += "  logBox.scrollTop = logBox.scrollHeight;";
    html += "  var datatosplit = event.data;";
    html += "  var datasplit = datatosplit.split(',');";
    html += "  console.log('Received ', datasplit);";
    html += "  var xValue = parseFloat(datasplit[0]);";
    html += "  var yValue = parseFloat(datasplit[1]);";
    html += "  console.log('Received X:', xValue, 'Y:', yValue);";
   html += "  var dot = document.createElement('div');";
html += "  dot.style.position = 'absolute';";
html += "  dot.style.width = '5px';";
html += "  dot.style.height = '5px';";
html += "  dot.style.borderRadius = '50%';";
html += "  dot.style.backgroundColor = '#FF0000';";

// Adjust the scaling factor based on the size of the xyPlane
html += "  var scaleFactor = xyPlane.offsetWidth / 800;"; // Assuming the xyPlane is 800x800 pixels

html += "  dot.style.left = (xValue *4* scaleFactor + xyPlane.offsetWidth / 2) + 'px';";
html += "  dot.style.top = (-yValue *4* scaleFactor + xyPlane.offsetHeight / 2) + 'px';"; // Note the negative sign for the y-axis
html += "  xyPlane.appendChild(dot);";
    html += "};";
    html += "</script>";

    html += "</body>";
    html += "</html>";

    server.send(200, "text/html", html);
}

void handleAPI() {
  if (server.method() == HTTP_POST) {
    String xValue = server.arg("x");
    String yValue = server.arg("y");

    // Process the received values
    int xcord = xValue.toInt();
    int ycord = yValue.toInt();

    // Send a response back
    String response = "Received X: " + String(xcord) + ", Y: " + String(ycord);
    server.send(200, "text/plain", response);

    Serial.println("Received X: " + String(xcord) + ", Y: " + String(ycord));
  }
}

void handleWebSocket(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  // Handle WebSocket events here
}

void sendLogData() {
  unsigned int count;

  // Get the current time
  unsigned long currentTime = millis();
  unsigned int seconds = (currentTime / 1000) % 60;
  unsigned int minutes = (currentTime / 60000) % 60;
  unsigned int hours = (currentTime / 3600000) % 24;

  String currentTimeString = "";
  if (hours < 10) {
    currentTimeString += "0";
  }
  currentTimeString += String(hours) + ":";
  if (minutes < 10) {
    currentTimeString += "0";
  }
  currentTimeString += String(minutes) + ":";
  if (seconds < 10) {
    currentTimeString += "0";
  }
  currentTimeString += String(seconds);

  // Generate random coordinates for demonstration
  float x = random(-100, 100);
  float y = random(-100, 100);
  float array[2];
  testingdata += String(x);
  testingdata += ",";
  testingdata += String(y);
  // count = 0;
  // for(int i = 0; i<2; i++){
  //   count= random(0,100);
  //   array[i] = float(count);
  // };
  // int counter = 0;
  // for(int j = 0; j<2; j ++){
  //   counter ++;
  //   testingdata += String(array[j]);
  //   if(counter == 1){
  //       testingdata += " , ";}
  //   else {counter =0;}
  // }

  // Add the log data to the string
  //logData = "<div>" + currentTimeString + "  --  X : " + String(x) + " ,  Y : " + String(y) + "</div>";
 //logData = testingdata;
 //xyPlaneData += String(x) + "," + String(y) + ";";
  // Send the log data to all connected WebSocket clients
  webSocketServer.broadcastTXT(testingdata);
  testingdata = "";
  count = 0;
}

void setup() {
  Serial.begin(115200);
  WiFi.config(staticIP, gateway, subnet); // Set the static IP address
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/api", handleAPI);

  webSocketServer.begin();
  webSocketServer.onEvent(handleWebSocket);

  server.begin();
  Serial.println("Server started");

  // ... Rest of the code remains the same ...
}

void loop() {
  server.handleClient();
  webSocketServer.loop();

  unsigned long currentMillis = millis();
  static unsigned long previousMillis = 0;
  const unsigned long interval = 1000; // 1 seconds

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    //counter++;
    //if(counter <7) {
      sendLogData();
      //}
  }
}
// #include <WiFi.h>
// #include <WebServer.h>
// #include <WebSocketsServer.h>
// #include <FS.h>
// #include <SPIFFS.h>

// int counter = 0;
// // GWN97CF78
// // kAGzjJJ5

// // ESPtesting
// String testingdata = "";
// const char *ssid = "ESPtesting";
// const char *password = "ESPtesting";
// IPAddress staticIP(192, 168, 137, 100); // Replace with your desired static IP address
// IPAddress gateway(192, 168, 137, 1);    // Replace with your network gateway IP address
// IPAddress subnet(255, 255, 255, 0);     // Replace with your network subnet mask

// WebServer server(80);
// WebSocketsServer webSocketServer(81);

// String logData = "";
// String xyPlaneData = "";

// void handleRoot()
// {
//   File htmlFile = SPIFFS.open("/src/index.html", "r"); // Open the HTML file
//   if (htmlFile)
//   {
//     size_t fileSize = htmlFile.size();
//     std::unique_ptr<char[]> buf(new char[fileSize]);
//     htmlFile.readBytes(buf.get(), fileSize);
//     server.send(200, "text/html", buf.get()); // Send the HTML content as the response
//     htmlFile.close();
//   }
//   else
//   {
//     server.send(500, "text/plain", "Error reading HTML file");
//   }
// }

// void handleAPI()
// {
//   if (server.method() == HTTP_POST)
//   {
//     String xValue = server.arg("x");
//     String yValue = server.arg("y");

//     // Process the received values
//     int xcord = xValue.toInt();
//     int ycord = yValue.toInt();

//     // Send a response back
//     String response = "Received X: " + String(xcord) + ", Y: " + String(ycord);
//     server.send(200, "text/plain", response);

//     Serial.println("Received X: " + String(xcord) + ", Y: " + String(ycord));
//   }
// }

// void handleWebSocket(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
// {
//   // Handle WebSocket events here
// }

// void sendLogData()
// {
//   unsigned int count;

//   // Get the current time
//   unsigned long currentTime = millis();
//   unsigned int seconds = (currentTime / 1000) % 60;
//   unsigned int minutes = (currentTime / 60000) % 60;
//   unsigned int hours = (currentTime / 3600000) % 24;

//   String currentTimeString = "";
//   if (hours < 10)
//   {
//     currentTimeString += "0";
//   }
//   currentTimeString += String(hours) + ":";
//   if (minutes < 10)
//   {
//     currentTimeString += "0";
//   }
//   currentTimeString += String(minutes) + ":";
//   if (seconds < 10)
//   {
//     currentTimeString += "0";
//   }
//   currentTimeString += String(seconds);

//   // Generate random coordinates for demonstration
//   float x = random(0, 100);
//   float y = random(0, 100);
//   float array[2];
//   testingdata += String(x);
//   testingdata += ",";
//   testingdata += String(y);
//   // count = 0;
//   // for(int i = 0; i<2; i++){
//   //   count= random(0,100);
//   //   array[i] = float(count);
//   // };
//   // int counter = 0;
//   // for(int j = 0; j<2; j ++){
//   //   counter ++;
//   //   testingdata += String(array[j]);
//   //   if(counter == 1){
//   //       testingdata += " , ";}
//   //   else {counter =0;}
//   // }

//   // Add the log data to the string
//   // logData = "<div>" + currentTimeString + "  --  X : " + String(x) + " ,  Y : " + String(y) + "</div>";
//   // logData = testingdata;
//   // xyPlaneData += String(x) + "," + String(y) + ";";
//   //  Send the log data to all connected WebSocket clients
//   webSocketServer.broadcastTXT(testingdata);
//   testingdata = "";
//   count = 0;
// }

// void setup() {
//     Serial.begin(115200);
//     WiFi.config(staticIP, gateway, subnet);
//     WiFi.begin(ssid, password);
//     while (WiFi.status() != WL_CONNECTED) {
//         delay(1000);
//         Serial.println("Connecting to WiFi...");
//     }
//     Serial.println("Connected to WiFi");
//     Serial.print("IP Address: ");
//     Serial.println(WiFi.localIP());

//     if (!SPIFFS.begin()) {
//         Serial.println("An error occurred while mounting SPIFFS");
//         return;
//     }

//     server.on("/", handleRoot);
//     server.on("/api", handleAPI);

//     webSocketServer.begin();
//     webSocketServer.onEvent(handleWebSocket);

//     server.begin();
//     Serial.println("Server started");
// }


// void loop()
// {
//   server.handleClient();
//   webSocketServer.loop();

//   unsigned long currentMillis = millis();
//   static unsigned long previousMillis = 0;
//   const unsigned long interval = 1000; // 1 seconds

//   if (currentMillis - previousMillis >= interval)
//   {
//     previousMillis = currentMillis;
//     // counter++;
//     // if(counter <7) {
//     sendLogData();
//     //}
//   }
// }


// #include <Servo.h>
// Servo servo1;
// int servoPin = 14;

// void setup(){ 
//   Serial.begin(115200);
//   servo1.attach(servoPin);
// }

// void loop(){
//   Serial.println("switch");
//   servo1.write(0);
//   delay(500);
//   servo1.write(90);
//   delay(500);
// }

// #include "Wire.h" // This library allows you to communicate with I2C devices.

// const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.

// int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
// int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
// int16_t temperature; // variables for temperature data

// char tmp_str[7]; // temporary variable used in convert function

// char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
//   sprintf(tmp_str, "%6d", i);
//   return tmp_str;
// }

// void setup() {
//   Serial.begin(9600);
//   Wire.begin();
//   Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
//   Wire.write(0x6B); // PWR_MGMT_1 register
//   Wire.write(0); // set to zero (wakes up the MPU-6050)
//   Wire.endTransmission(true);
// }
// void loop() {
//   Wire.beginTransmission(MPU_ADDR);
//   Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
//   Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
//   Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
  
//   // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
//   accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
//   accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
//   accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
//   temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
//   gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
//   gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
//   gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
  
//   // print out data
//   Serial.print("aX = "); Serial.print(convert_int16_to_str(accelerometer_x));
//   Serial.print(" | aY = "); Serial.print(convert_int16_to_str(accelerometer_y));
//   Serial.print(" | aZ = "); Serial.print(convert_int16_to_str(accelerometer_z));
//   // the following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]
//   Serial.print(" | tmp = "); Serial.print(temperature/340.00+36.53);
//   Serial.print(" | gX = "); Serial.print(convert_int16_to_str(gyro_x));
//   Serial.print(" | gY = "); Serial.print(convert_int16_to_str(gyro_y));
//   Serial.print(" | gZ = "); Serial.print(convert_int16_to_str(gyro_z));
//   Serial.println();
  
//   // delay
//   delay(1000);
// }