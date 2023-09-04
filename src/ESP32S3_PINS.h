// This Header is used to set the pin numbers and set the includes

// Inlcudes
#include <Arduino.h>
#include <pwmWrite.h>
#include <Connection.h>
#include <string>
#include <iostream>
#include <Wire.h>
#include <ERROR.h>

#include <Esp.h>

// Libaries
#include <EasyRobot.h>
#include <TFLI2C.h>
#include <ESP32Servo.h>



const float WHEEL_CIRCUMFERENCE = 153.15;//157.1; // Dia = 48.75
const float WHEEL_DISTANCE = 311.6; //281.6; // OUTER: 311.6mm INNER: 281.6mm 
const int MICROSTEP = 4;
const int STEPPER_STEP_COUNT = 200;
const int GEAR_RATIO = 2;
// const int STEPS_PER_REV;

//  I2C
#define I2CA Wire
#define I2CB Wire1
const int I2CA_SDA = 8;
const int I2CA_SCL = 9;
const int I2CB_SDA = 1;
const int I2CB_SCL = 2;

//  LUNA
const int16_t LiDAR_ADD_1 = 0x10;  // Straight Ahead
const int16_t LiDAR_ADD_2 = 0x25; // Swivel motor
const int16_t LiDAR_ADD_3 = 0x15; //3rd Luna
uint16_t LiDAR_frame_rate = 0x00;
const int LiDAR_1_signal = 48;
const int LiDAR_2_signal = 47;
const int LiDAR_3_signal = 21;

// Motor Pins
// S2
const int R_Stepper_STEP_PIN = 17;
const int R_Stepper_DIR_PIN = 18;
const int R_Stepper_ENABLE_PIN = 16;

// S1
const int L_Stepper_STEP_PIN = 15;
const int L_Stepper_DIR_PIN = 7;
const int L_Stepper_ENABLE_PIN = 6;

const int MS1_pin = 42;
const int MS2_pin = 41;
const int MS3_pin = 40;


// Slave MCU
const uint8_t slave_ADDR = 0x55;


//  Line Sensors
const int IR1_PIN = 14;
const int IR2_PIN = 10;
const int IR3_PIN = 38;
const int IR4_PIN = 39;

//  Limit Switches
const int LIM1_PIN = 4;
const int LIM2_PIN = 5;



//  SPI
const int MISO_PIN = 13;
const int MOSI_PIN = 11;
const int SPI_SCK_PIN = 12;


