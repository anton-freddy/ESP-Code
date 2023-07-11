#include <Arduino.h>
#include <FlexyStepperCustom.h>
#include <math.h>
#include <ArduinoQueue.h>

enum unit
{
  KMH = 1,
  MMS = 2
};

struct pos
{
  long x_pos;
  long y_pos;
  double a_pos;
};

struct motor_move
{
  long LeftMotor;
  long RightMotor;
};

struct move
{
  long target_x_pos;
  long traget_y_pos;
  double target_a_pos;
  bool is_rotate;

  move()
  {
    target_a_pos = 0;
    target_x_pos = 0;
    traget_y_pos = 0;
    is_rotate = false;
  }

  move(long x, long y)
  {
    target_a_pos = 0;
    target_x_pos = x;
    traget_y_pos = y;
    is_rotate = false;
  }
  move(double a)
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
  const float rotationConstant = 171.931; // 158.3;//160.49877; // Must be adjusted based on wheel distance and mounting points
  long x_pos;
  long y_pos;
  double a_pos;
  bool L_MOVE_DONE;
  bool R_MOVE_DONE;
  bool ROT_MOVE;
  bool STR_MOVE;

  // Process Movement Variables
  long previous_cord_LS;
  long previous_cord_RS;
  long current_cord_LS;
  long current_cord_RS;
  long target_cord_LS;
  long target_cord_RS;

  void updatePosition(long deltaX, long deltaY, double targetOrientation);
  void updatePosition(long deltaX, long deltaY);
  void updatePosition(double deltaOrientation);

  double calc_orientation(long target_X, long target_y);
  long calc_delta_Y(long straight_line_dist);
  long calc_delta_X(long straight_line_dist);
  long calc_diagonal_distance(long target_x, long target_y);

  void load_move(void);

  void setSpeedInKMH(float speed);
  void setSpeedInMMS(float speed);
  void setAccelerationInKMHH(float speed);
  void setAccelerationInMMSS(float speed);

  void sendError(String MSG);

public:
  FlexyStepper leftMotor;
  FlexyStepper rightMotor;

  EasyRobot();

  void begin(unit speed_units, float stepsPerMillimeters, float speed, float Acceleration);
  void setUpPins(int leftMotorStepPin, int leftMotorDirPin, int leftMotorEnablePin, int rightMotorStepPin, int rightMotorDirPin, int rightMotorEnablePin);

  void setPosition(double angle);
  void setPosition(long xPos, long yPos);
  void setPosition(long xPos, long yPos, double angle);

  void setUpMove(long target_x, long target_y);
  void setUpTurn(double TargetOrientation);
  void setUpTurnDeg(double TargetOrientation);
  long getXCoordinate() const;
  long getYCoordinate() const;
  double getOrientation() const;

  void moveTo(long targetX, long targetY);
  void setupMoveForward(long distance);
  bool processMovement();
  bool motionComplete();

  float toDeg(double rad);
  float toRad(double deg);

  void stop();

  bool add_move(long targetX, long targetY);
  bool add_move(double targetA);
};


