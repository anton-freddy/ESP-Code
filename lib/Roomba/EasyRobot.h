#include <Arduino.h>
#include <math.h>
#include <AS5600_Wire.h>
#include <AS5600_Wire1.h>
#include <Wire.h>
#include <ERROR.h>
#include <MPU9250.h>
//  Wheel Distance = OUTER: 311.6mm INNER: 281.6mm

// void send_ERROR(int error_code);

// void send_ERROR(int error_code);

const long CORD_REFRESH_RATE = 100; //  Rate at which the cordinates are updated

enum MoveState
{
  IDLE,
  TURN,
  STRAIGHT,
  BACK,
  ERROR,
  BACK_OFF,
  RUNNING,
  WAIT
};

enum unit
{
  KMH = 1,
  MMS = 2
};

enum motor
{
  left = 1,
  right = 2,
  both = 0
};

struct pos
{
  float x_pos;
  float y_pos;
  float a_pos;
};

struct motor_move
{
  float LeftMotor;
  float RightMotor;
};

struct Move
{
  float x;
  float y;
  Move *next;
};

class EasyRobot
{

private:
  Move *head = nullptr;
  Move *tail = nullptr;
  

  AMS_5600_Wire R_ENCODER;
  AMS_5600_Wire1 L_ENCODER;

  //  Stepper Varibales
  byte L_E_pin;
  byte L_S_pin;
  byte L_D_pin;

  byte R_E_pin;
  byte R_S_pin;
  byte R_D_pin;

  byte MS1_pin = 0;
  byte MS2_pin = 0;
  byte MS3_pin = 0;

  
  long acceleration;
  float wheel_circumfrence_mm;
  int steps_per_rev;
  float wheel_distance_mm;
  float steps_per_mm;
  int micro_step;
  int stepper_steps_per_rev;
  int gear_ratio;

  //  Tolerance Variables
  const float DistanceTolerance = 40;
  const float ThetaTolernce = 0.05;

  float targetX = 0.0;
  float targetY = 0.0;

  float L_ENC_PREVIOUS = 0;
  float L_ENC_TOTAL_REV = 0;

  long L_direction = 0;
  long L_CurrentSpeed = 0;
  long L_Speed_SPS = 0;
  
  long L_step_time = 0;
  bool L_STEP_MOVING = false;


  float R_ENC_PREVIOUS = 0;
  float R_ENC_TOTAL_REV = 0;
  long R_step_time = 0;
  long R_direction = 0;
  long R_CurrentSpeed = 0;
  long R_Speed_SPS = 0;
  
  
  bool R_STEP_MOVING = false;

  float leftWheelSpeed = 2000;
  float rightWheelSpeed = 2000;

  //  Timing Variables  //------------
  unsigned long currentMillis_pos_update = 0;
  unsigned long previousMillis_pos_update = 0;
  unsigned long L_previous_time = 0;
  unsigned long R_previous_time = 0;
  unsigned long pose_previousTime = 0;
  unsigned long encoder_current_millis = 0;
  unsigned long encoder_previous_millis = 0;


  const float rotationConstant = 171.931 / 10; // 158.3;//160.49877; // Must be adjusted based on wheel distance and mounting points

  //  Position Variables  //--------
  volatile float x_pos = 0;
  volatile float y_pos = 0;
  volatile float a_pos = 0;
  pos target;
  float L_prev_dist = 0;
  float R_prev_dist = 0;
  

  bool L_MOVE_DONE;
  bool R_MOVE_DONE;
  bool ROT_MOVE = false;
  bool STR_MOVE = false;

  // Process Movement Variables
  float previous_cord_LS = 0;
  float previous_cord_RS = 0;
  float current_cord_LS = 0;
  float current_cord_RS = 0;
  float target_cord_LS = 0;
  float target_cord_RS = 0;

  // Stepper Functions
  void L_setSpeed_SPS(long SPS);
  void R_setSpeed_SPS(long SPS);
  void L_setSpeed_MMPS(float MMPS);
  void R_setSpeed_MMPS(float MMPS);
  void setAcceleration_SPSPS(long SPSPS);
  void L_setTarget_POS(float Target_MM);
  void R_setTarget_POS(float Target_MM);
  bool L_stepper_target_reached();
  bool R_stepper_target_reached();
  void setSpeedInKMH(float speed);
  
  void setAccelerationInKMHH(float speed);
  void setAccelerationInMMSS(float speed);
  void resumeStepper(motor select);
  void stopStepper(motor select);

  //  Positioning   //-----------
  void updatePosition(float deltaX, float deltaY, float targetOrientation);
  void updatePosition(float deltaX, float deltaY);
  void updatePosition(float deltaOrientation);

  float calc_orientation(float target_X, float target_y);
  float calc_delta_Y(float straight_line_dist);
  float calc_delta_X(float straight_line_dist);
  float calc_diagonal_distance(float target_x, float target_y);
  float get_delta_theta(float target);



public:

  MPU9250 mpu;

  volatile float L_Current_POS_MM = 0;
 volatile float L_Target_POS_MM = 0;
volatile float R_Current_POS_MM = 0;
 volatile float R_Target_POS_MM = 0;

  float defaultSpeed_MMS = 0;

  bool obsticale_avoiding = false;
  float obsticale_prev_orientation;
  float obsticale_xpos;
  float obsticale_ypos;

 MoveState movementState = IDLE;

  void setSpeedInMMS(float speed);


  EasyRobot(float wheel_circumfrence, float wheel_distance, int MICRO_STEP, int STEPPER_STEP_COUNT, int GEAR_RATIO);
  void loop();
  //  Move Buffer   //-----------
  void enqueueMove(float x, float y);
  void insertMoveBeforeCurrent(float x, float y);
  bool executeMoves();
  void clearMoves();

  //  Positioning   //-----------
  void updatePose();
  void setPosition(float angle);
  void setPosition(float xPos, float yPos);
  void setPosition(float xPos, float yPos, float angle);

  void update_stepper_DIR_pin();
  void moveSteppers();
  void enableStepper(motor select);
  void disableStepper(motor select);

  void UpdatePosFromEncoders(long refresh_rate);
  void calibrateMPU();
  void print_MPU_calibration();

  
  void followHeading(float heading, float speedMMS);

  void begin(unit speed_units, float speed);
  void setUpMotors(byte leftMotorStepPin, byte leftMotorDirPin, byte leftMotorEnablePin, byte rightMotorStepPin, byte rightMotorDirPin, byte rightMotorEnablePin, byte MS1Pin, byte MS2Pin, byte MS3Pin);
  void setUpMotors(byte leftMotorStepPin, byte leftMotorDirPin, byte leftMotorEnablePin, byte rightMotorStepPin, byte rightMotorDirPin, byte rightMotorEnablePin);
  void setUpEncoders();
  void resetEncoders(motor selector);
  float getEncoderAngle(motor identifier);



  void setUpMove(float target_x, float target_y);
  void setUpTurn(float TargetOrientation);
  void setUpTurnDeg(float TargetOrientation);
  float getXCoordinate() const;
  float getYCoordinate() const;
  float getOrientation() const;

  float L_getCurrentPos_CM(void);
  float L_getCurrentSpeed_MMS(void);
  float R_getCurrentPos_CM(void);
  float R_getCurrentSpeed_MMS(void);

  void moveTo(float targetX, float targetY);
  void setupMoveForward(float distance);
  bool processMovement();
  bool motionComplete();

  float toDeg(float rad);
  float toRad(float deg);

  void stop();
  void resume();

  bool add_move(float targetX, float targetY);
  bool add_move(float targetA);


  unsigned long backOff_current_millis = 0;
  unsigned long backOff_previous_millis = 0;
};
