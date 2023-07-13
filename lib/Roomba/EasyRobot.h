#include <Arduino.h>
#include <FlexyStepperCustom.h>
#include <math.h>
#include <ArduinoQueue.h>
//  Wheel Distance = OUTER: 311.6mm INNER: 281.6mm 
//  Wheel Circumference = 157.1mm
enum unit
{
  KMH = 1,
  MMS = 2
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

  long L_CurrentSteps = 0;
  long L_TargetSteps = 0;
  long L_direction = 0;
  long L_CurrentSpeed = 0;
  long L_TargetSpeed = 0;
  long L_previous_time = 0;
  long L_step_time = 0;

  long R_CurrentSteps = 0;
  long R_TargetSteps = 0;
  long R_direction = 0;
  long R_CurrentSpeed = 0;
  long R_TargetSpeed = 0;
  long R_previous_time = 0;
  long R_step_time = 0;


  const float rotationConstant = 171.931; // 158.3;//160.49877; // Must be adjusted based on wheel distance and mounting points
  float x_pos;
  float y_pos;
  float a_pos;
  bool L_MOVE_DONE;
  bool R_MOVE_DONE;
  bool ROT_MOVE;
  bool STR_MOVE;

  // Process Movement Variables
  float previous_cord_LS;
  float previous_cord_RS;
  float current_cord_LS;
  float current_cord_RS;
  float target_cord_LS;
  float target_cord_RS;

  // Stepper Functions
  void L_setSpeed_SPS(long SPS);
  void R_setSpeed_SPS(long SPS);
  void L_setSpeed_MMPS(float MMPS);
  void R_setSpeed_MMPS(float MMPS);
  void setAcceleration_SPSPS(long SPSPS);
  float L_getCurrentPos_MM(void);
  float L_getCurrentSpeed_MMS(void);
  float R_getCurrentPos_MM(void);
  float R_getCurrentSpeed_MMS(void);

  void L_setTarget_MM(float Target_MM);
  void L_setTarget_STEP(long Target_STEP);
  void R_setTarget_MM(float Target_MM);
  void R_setTarget_STEP(long Target_STEP);


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

  void sendError(String MSG);

public:

  bool moveSteppers();

  FlexyStepper leftMotor;
  FlexyStepper rightMotor;

  EasyRobot(int steps_per_rev, float wheel_circumfrence, float wheel_distance);

  void begin(unit speed_units, float speed, float Acceleration);
  void setUpPins(byte leftMotorStepPin, byte leftMotorDirPin, byte leftMotorEnablePin, byte rightMotorStepPin, byte rightMotorDirPin, byte rightMotorEnablePin);

  void setPosition(float angle);
  void setPosition(float xPos, float yPos);
  void setPosition(float xPos, float yPos, float angle);

  void setUpMove(float target_x, float target_y);
  void setUpTurn(float TargetOrientation);
  void setUpTurnDeg(float TargetOrientation);
  float getXCoordinate() const;
  float getYCoordinate() const;
  float getOrientation() const;

  void moveTo(float targetX, float targetY);
  void setupMoveForward(float distance);
  bool processMovement();
  bool motionComplete();

  float toDeg(float rad);
  float toRad(float deg);

  void stop();

  bool add_move(float targetX, float targetY);
  bool add_move(float targetA);
};


