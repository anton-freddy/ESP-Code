#include <Arduino.h>
#include <FlexyStepperCustom.h>
#include <FastAccelStepper.h>
#include <math.h>
#include <ArduinoQueue.h>
#include <AS5600_Wire.h>
#include <AS5600_Wire1.h>
#include <Wire.h>
#include <ERROR.h>
//  Wheel Distance = OUTER: 311.6mm INNER: 281.6mm 

//void send_ERROR(int error_code);

void send_ERROR(int error_code);



const long CORD_REFRESH_RATE = 100; //  Rate at which the cordinates are updated
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

struct move
{
  float target_x_pos;
  float traget_y_pos;
  float target_a_pos;
  bool is_rotate;

  move()
  {
    target_a_pos = 0;
    target_x_pos = 0;
    traget_y_pos = 0;
    is_rotate = false;
  }

  move(float x, float y)
  {
    target_a_pos = 0;
    target_x_pos = x;
    traget_y_pos = y;
    is_rotate = false;
  }
  move(float a)
  {
    target_a_pos = a;
    target_x_pos = 0;
    traget_y_pos = 0;
    is_rotate = true;
  }
};




class EasyRobot
{

private:

AMS_5600_Wire R_ENCODER;
AMS_5600_Wire1 L_ENCODER;

  //  Stepper Varibales
  byte L_E_pin;
  byte L_S_pin;
  byte L_D_pin;

  byte R_E_pin;
  byte R_S_pin;
  byte R_D_pin;

 

  long acceleration;
  float wheel_circumfrence_mm;
  int steps_per_rev;
  float wheel_distance_mm;
  float steps_per_mm;
  int micro_step;
  int stepper_steps_per_rev;
  int gear_ratio;

  long encoder_current_millis = 0;
  long encoder_previous_millis = 0;

  volatile float L_Current_POS_CM = 0;
  volatile float L_Target_POS_CM = 0;
  float L_ENC_PREVIOUS = 0;
  float L_ENC_TOTAL_REV = 0;


  long L_direction = 0;
  long L_CurrentSpeed = 0;
  long L_Speed_SPS = 0;
  long L_previous_time = 0;
  long L_step_time = 0;
  bool L_STEP_MOVING = false;

  volatile float R_Current_POS_CM = 0;
  volatile float R_Target_POS_CM = 0;
  float R_ENC_PREVIOUS = 0;
  float R_ENC_TOTAL_REV = 0;

  long R_direction = 0;
  long R_CurrentSpeed = 0;
  long R_Speed_SPS = 0;
  long R_previous_time = 0;
  long R_step_time = 0;
  bool R_STEP_MOVING = false;

  long currentMillis_pos_update = 0;
  long previousMillis_pos_update = 0;

  const float rotationConstant = 171.931/10; // 158.3;//160.49877; // Must be adjusted based on wheel distance and mounting points
  float x_pos;
  float y_pos;
  float a_pos;
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



  void updatePosition(float deltaX, float deltaY, float targetOrientation);
  void updatePosition(float deltaX, float deltaY);
  void updatePosition(float deltaOrientation);

  float calc_orientation(float target_X, float target_y);
  float calc_delta_Y(float straight_line_dist);
  float calc_delta_X(float straight_line_dist);
  float calc_diagonal_distance(float target_x, float target_y);

  void load_move(void);

  void setSpeedInKMH(float speed);
  void setSpeedInMMS(float speed);
  void setAccelerationInKMHH(float speed);
  void setAccelerationInMMSS(float speed);


public:
    void update_stepper_DIR_pin();
  void moveSteppers();
  void enableStepper(motor select);
  void disableStepper(motor select);
  EasyRobot(float wheel_circumfrence, float wheel_distance, int MICRO_STEP, int STEPPER_STEP_COUNT, int GEAR_RATIO);
  void UpdatePosFromEncoders(long refresh_rate);

  void begin(unit speed_units, float speed, float Acceleration);
  void setUpMotors(byte leftMotorStepPin, byte leftMotorDirPin, byte leftMotorEnablePin, byte rightMotorStepPin, byte rightMotorDirPin, byte rightMotorEnablePin);
  void setUpEncoders(byte L_ENC_SDA, byte L_ENC_SCL, byte R_ENC_SDA, byte R_ENC_SCL);
  void resetEncoders(motor selector);
  float getEncoderAngle(motor identifier);

  void setPosition(float angle);
  void setPosition(float xPos, float yPos);
  void setPosition(float xPos, float yPos, float angle);

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
};


