#include <stdio.h>
#include <Arduino.h>
#include <Servo.h>

#define SERVO_PIN 17

Servo myservo;

void setup() {
    pinMode(SERVO_PIN, OUTPUT);
    Serial.begin(115200);
    myservo.attach(SERVO_PIN);

}

int pos = 0;

void loop() {
  analogWrite(SERVO_PIN, 50);
}