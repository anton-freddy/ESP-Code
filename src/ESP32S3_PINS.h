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

//  LUNA
#define LiDAR_I2C Wire
const int LiDAR_SCL_PIN = 17;
const int LiDAR_SDA_PIN = 18;
const int16_t LiDAR_ADD_1 = 0x10;  // Straight Ahead
const int16_t LiDAR_ADD_2 = 0x25; // Swivel motor
const int16_t LiDAR_ADD_3 = 0x15; //3rd Luna
uint16_t LiDAR_frame_rate = FPS_250;

// Motor Pins
// S2
const int R_Stepper_STEP_PIN = 9;
const int R_Stepper_DIR_PIN = 46;
const int R_Stepper_ENABLE_PIN = 10;
const int R_Stepper_ENCODER_PIN = 47;

const int R_ENC_SDA = LiDAR_SDA_PIN;
const int R_ENC_SCL = LiDAR_SCL_PIN;
// S1
const int L_Stepper_STEP_PIN = 7;
const int L_Stepper_DIR_PIN = 6;
const int L_Stepper_ENABLE_PIN = 15;
const int L_Stepper_ENCODER_PIN = 21;

const int L_ENC_SDA = 1;
const int L_ENC_SCL = 2;

// Servo
const int SERVO_PIN = 16;

//  DC_Motor
const int L_DC_PWM_PIN = 0;
const int L_DC_IN1_PIN = 38;
const int L_DC_IN2_PIN = 37;

const int R_DC_PWM_PIN = 45;
const int R_DC_IN1_PIN = 36;
const int R_DC_IN2_PIN = 35;

const int DC_STBY_PIN = 48;


//  Line Sensors
const int IR1_PIN = 14;
const int IR2_PIN = 13;
const int IR3_PIN = 12;
const int IR4_PIN = 11;

//  Limit Switches
const int LIM1_PIN = 5;
const int LIM2_PIN = 4;



//  SPI
const int MISO_PIN = 39;
const int MOSI_PIN = 40;
const int SPI_SCK_PIN = 41;
const int SD_CS_PIN = 42;

//  RESET
const int RST_PIN = 20;

