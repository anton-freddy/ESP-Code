// This Header is used to set the pin numbers and set the includes

// Inlcudes
#include <Arduino.h>
#include <pwmWrite.h>
#include <Connection.h>
#include <string>
#include <iostream>
#include <Wire.h>
#include <ERROR.h>
#include <web_connection.h>
#include <Battery.h>

#include <Esp.h>

// Libaries
#include <EasyRobot.h>
#include <ESP32Servo.h>
#include <SPIFFS.h>
#include <LiDAR.h>


const float WHEEL_CIRCUMFERENCE = 153.15; // 157.1; // Dia = 48.75
const float WHEEL_DISTANCE = 311.6;       // 281.6; // OUTER: 311.6mm INNER: 281.6mm
const int MICROSTEP = 2;
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
const int16_t LiDAR_ADD_1 = 0x10; // Straight Ahead
const int16_t LiDAR_ADD_2 = 0x25; // Swivel motor
const int16_t LiDAR_ADD_3 = 0x15; // 3rd Luna
uint16_t LiDAR_frame_rate = 0x00;
const int LiDAR_1_signal = 48;
const int LiDAR_2_signal = 47;
const int LiDAR_3_signal = 21;

#define ENABLE_LiDAR1
#define ENABLE_LiDAR2
//#define ENABLE_LiDAR3

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
const int L_IR_PIN = 14;
const int BATTERY_LEVEL_PIN = 10;    // IR2 on board pinout
const int C_BUMP_PIN = 38; // IR3 on board pinout
const int R_IR_PIN = 39;    // IR4 on board pinout

Battery batt(8000, 13000, BATTERY_LEVEL_PIN);//3041

//  Limit Switches
const int R_BUMP_PIN = 4; // LIM1 on board pinout
const int L_BUMP_PIN = 5; // LIM2 on board pinout

const int obstciale_radius = 200;




//  SPI
const int MISO_PIN = 13;
const int MOSI_PIN = 11;
const int SPI_SCK_PIN = 12;

//  Time Variables
unsigned long previousMillis_Servo = 0;

//  Servo Var
  int servo_increment = 5;
  int servo_pos = 90;
  int servo_delayBetweenSteps = 20;

float map_f(float x, float in_min, float in_max, float out_min, float out_max)
{
  const float run = in_max - in_min;
  if (run == 0)
  {
    log_e("map(): Invalid input range, min == max");
    return -1; // AVR returns -1, SAM returns 0
  }
  const float rise = out_max - out_min;
  const float delta = x - in_min;
  return (delta * rise) / run + out_min;
}

void float_to_ints(float floatValue, int &integerPart, int &decimalPart)
{
  // Split the float into integer and decimal parts
  integerPart = int(floatValue);
  decimalPart = int((floatValue - integerPart) * 1000); // Multiply by 1000 to get 3 decimal places
}

float ints_to_float(int integerPart, int decimalPart)
{
  // Reconstruct the float from integer and decimal parts
  return float(integerPart) + float(decimalPart) / 1000.0; // Divide by 1000 to restore the decimal places
}


enum RegisterAddress
{
    REG_SERVO_POS,
    REG_SERVO_STATE,
    REG_CHARGE_LEVEL,
    REG_XPOS,
    REG_YPOS,
    REG_APOS,
    REG_DCL,
    REG_DCR
};

enum ServoState
{
    CENTER = 0,
    RUN = 1,
    PAUSE = 2
};

bool setup_I2C();
void setup_IR();
bool get_IR1_status();
bool get_IR2_status();
void IR1_ISR();
void IR2_ISR();
void setupBatterySense();
int getBatteryLevel();
void bump_ISR_C();
void bump_ISR_L();
void bump_ISR_R();
void setupBumpers();
void setBumpBackOFF();
void tickServo();

bool bump_triggred = false;
Servo servo;
bool SERVO_CLOCKWISE = true;
int SERVO_pos = 0;
long servo_previousMillis = 0;
#define SERVO_INTERVAL 200
bool is_I2C_setup = false;
void setup_servo();
void move_servo();

int requestIntFromSlave(uint8_t regAddress);
float requestFloatFromSlave(uint8_t regAddress);

void writeIntToSlave(uint8_t regAddress, int data);
void writeFloatToSlave(uint8_t regAddress, float data);