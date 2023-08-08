#include <Arduino.h>
#include <iostream>
#include <math.h>

#define LiDAR_1 0
#define LiDAR_2 1
#define LiDAR_3 2
#define LEFT_ENCODER 3
#define RIGHT_ENCODER 4
#define ENCODERS 34
#define SERVO 5
#define LEFT_STEPPER 6
#define RIGHT_STEPPER 7
#define MOVEMENT 8
#define OLED_DISPLAY 9
#define IR1 10
#define IR2 11
#define IR3 12
#define IR4 13

void send_ERROR(int module_code, int error_code);

String getLiDAR_ERRORS(int module_code, int error_code);
String getStepper_ERRORS(int module_code, int error_code);
String getEncoder_ERRORS(int module_code, int error_code);
String getMovement_ERRORS(int module_code, int error_code);
String getIR_ERRORS(int module_code, int error_code);

void serial_send_ERROR(String MSG);
