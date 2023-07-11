#include <EasyRobot.h>

ArduinoQueue<move> move_q(500);

//--  Constructor
EasyRobot::EasyRobot()
{
  current_cord_LS = 0;
  current_cord_RS = 0;
}

//--  Private Functions ----------------------------------------

// Updates the position of the robot, values passed should be delta values, NOT target values
void EasyRobot::updatePosition(long deltaX, long deltaY, double deltaOrientation)
{
  x_pos += deltaX;
  y_pos += deltaY;
  a_pos += deltaOrientation;
    if (a_pos < 0)
  {
    a_pos = a_pos + (2 * PI);
  }
  else if (a_pos > (2 * PI))
  {
    a_pos = a_pos - (2 * PI);
  }
  else if (a_pos == (2 * PI))
  {
    a_pos = 0;
  }
}

// Updates the position of the robot, values passed should be delta values, NOT target values
void EasyRobot::updatePosition(long deltaX, long deltaY)
{
  x_pos += deltaX;
  y_pos += deltaY;
}

// Updates the position of the robot, values passed should be delta values, NOT target values
void EasyRobot::updatePosition(double deltaOrientation)
{
  a_pos += deltaOrientation;
  if (a_pos < 0)
  {
    a_pos = a_pos + (2 * PI);
  }
  else if (a_pos > (2 * PI))
  {
    a_pos = a_pos - (2 * PI);
  }
  else if (a_pos == (2 * PI))
  {
    a_pos = 0;
  }
}

// Returns the target orinenation of the robot based on the change in X and Y
double EasyRobot::calc_orientation(long target_x, long target_y)
{
  long deltaX = target_x - x_pos;
  long deltaY = target_y - y_pos;
  // return atan2f(deltaY, deltaX);
  double target_orientation = 0;
  if (deltaX == 0)
  {
    if (deltaY > 0)
    {
      target_orientation = 0;
    }
    else
    {
      target_orientation = PI;
    }
  }
  else if (deltaY == 0)
  {
    if (deltaX > 0)
    {
      target_orientation = 0.5 * PI;
    }
    else
    {
      target_orientation = 1.5 * PI;
    }
  }
  else if (deltaX > 0)
  {
    if (deltaY > 0)
    {
      Serial.println("QUADRANT 1");
      target_orientation = atan(deltaX / deltaY);
    }
    else if (deltaY < 0)
    {
      Serial.println("QUADRANT 2");
      target_orientation = 0.5 * PI + atan(abs(deltaY / deltaX));
    }
    else
    {
      Serial.println("ERROR X is positive but Y is not matched");
    }
  }
  else if (deltaX < 0)
  {
    if (deltaY < 0)
    {
      Serial.println("QUADRANT 3");
      target_orientation = PI + atan(abs(deltaX / deltaY));
    }
    else if (deltaY > 0)
    {
      Serial.println("QUADRANT 4");
      target_orientation = 1.5 * PI + atan(abs(deltaY / deltaX));
    }
    else
    {
      Serial.println("ERROR X is negative but Y is not matched");
    }
  }
  else
  {
    Serial.println((String)deltaX);
    Serial.println((String)deltaY);
    Serial.println("ERROR NO SUITABLE a_pos FOUND");
  }

  if (target_orientation > 2 * PI)
  {
    Serial.println("a_pos OUT OF BOUNDS, ABOVE 2PI");
  }
  else if (target_orientation < 0)
  {
    Serial.println("a_pos OUT OF BOUNDS, below 0");
  }
  else
  {
    if (target_orientation == (2 * PI))
    {
      target_orientation = 0;
    }
  }
  return target_orientation;
}

// Returns the change in Ypos (deltaY), based on the strightline distance covered and the current a_pos of the robot
long EasyRobot::calc_delta_Y(long straight_line_dist)
{
  if (a_pos == 0)
  {
    return 0; // No change in X
  }
  else if (a_pos == (0.5 * PI))
  {
    return straight_line_dist; // No change in Y
  }
  else if (a_pos == PI)
  {
    return 0; // No chnage along X axis
  }
  else if (a_pos == 1.5 * PI)
  {
    return -straight_line_dist; // No change in Y
  }
  else if (a_pos > 0 && a_pos < (0.5 * PI))
  {
    return straight_line_dist * sinf(a_pos);
  }
  else if (a_pos > (0.5 * PI) && a_pos < PI)
  {
    return straight_line_dist * sinf(a_pos - (0.5 * PI));
  }
  else if (a_pos > PI && a_pos < (1.5 * PI))
  {
    return -(straight_line_dist * sinf(a_pos - PI));
  }
  else if (a_pos > (1.5 * PI) && a_pos < (2 * PI))
  {
    return -(straight_line_dist * sinf(a_pos - (1.5 * PI)));
  }
  else
  {
    Serial.println("ERROR! NO DELTA Y FOUND. D: " + (String)straight_line_dist + " A: " + (String)a_pos);
    return 0;
  }
}

// Returns the change in Xpos (deltaX), based on the strightline distance covered and the current a_pos of the robot
long EasyRobot::calc_delta_X(long straight_line_dist)
{
  if (a_pos == 0)
  {
    return 0; // No change in X
  }
  else if (a_pos == (0.5 * PI))
  {
    return straight_line_dist; // No change in Y
  }
  else if (a_pos == PI)
  {
    return 0; // No chnage along X axis
  }
  else if (a_pos == 1.5 * PI)
  {
    return -straight_line_dist; // No change in Y
  }
  else if (a_pos > 0 && a_pos < (0.5 * PI))
  {
    return straight_line_dist * cosf(a_pos);
  }
  else if (a_pos > (0.5 * PI) && a_pos < PI)
  {
    return straight_line_dist * cosf(a_pos - (0.5 * PI));
  }
  else if (a_pos > PI && a_pos < (1.5 * PI))
  {
    return -(straight_line_dist * cosf(a_pos - PI));
  }
  else if (a_pos > (1.5 * PI) && a_pos < (2 * PI))
  {
    return -(straight_line_dist * cosf(a_pos - (1.5 * PI)));
  }
  else
  {
    Serial.println("ERROR! NO DELTA X FOUND. D: " + (String)straight_line_dist + " A: " + (String)a_pos);
    return 0;
  }
}

// Returns the diagonal distance the robot needs to cover based on the target XY pos, current orientation
long EasyRobot::calc_diagonal_distance(long target_x, long target_y)
{
  long deltaX = target_x - x_pos;
  long deltaY = target_y - y_pos;
  long distance_diagonal;

  if (deltaX == 0)
  {
    distance_diagonal = deltaY;
  }
  else if (deltaY == 0)
  {
    distance_diagonal = deltaX;
  }
  else
  {
    distance_diagonal = sqrt(powf(deltaX, 2) + powf(deltaY, 2)); // abs(deltaX / sin(getOrientation())); // sqrt(pow(deltaX, 2) + pow(deltaY, 2)); // Need to add signed to quadrant 3 & 4
  }
  return distance_diagonal;
}

void EasyRobot::load_move(void)
{
  if (move_q.isEmpty())
  {
    return;
  }
  move next_move = move_q.getHead();
  if (!next_move.is_rotate)
  {
    next_move.target_a_pos = calc_orientation(next_move.target_x_pos, next_move.traget_y_pos);
    if (next_move.target_a_pos != a_pos)
    {
      setUpTurn(next_move.target_a_pos);
      return;
    }
    else
    {
      setUpMove(next_move.target_x_pos, next_move.traget_y_pos);
      move_q.dequeue();
      return;
    }
  }
  else
  {
    setUpTurn(next_move.target_a_pos);
    move_q.dequeue();
    return;
  }
  return;
}

// Sets the speed of the robot in KM/h, !!NOTE!! you must set the steps per millimeter before setting the speed
void EasyRobot::setSpeedInKMH(float speed)
{
  setSpeedInMMS(speed * 277.7777777778);
}

// Sets the speed of the robot in mm/s, !!NOTE!! you must set the steps per millimeter before setting the speed
void EasyRobot::setSpeedInMMS(float speed)
{

  leftMotor.setSpeedInMillimetersPerSecond(speed);
  rightMotor.setSpeedInMillimetersPerSecond(speed);
}

// Sets the speed of the robot in KM/h, !!NOTE!! you must set the steps per millimeter before setting the speed
void EasyRobot::setAccelerationInKMHH(float speed)
{
  setAccelerationInMMSS(speed * 0.2778);
}

// Sets the speed of the robot in mm/s, !!NOTE!! you must set the steps per millimeter before setting the speed
void EasyRobot::setAccelerationInMMSS(float speed)
{

  leftMotor.setAccelerationInMillimetersPerSecondPerSecond(speed);
  rightMotor.setAccelerationInMillimetersPerSecondPerSecond(speed);
}

void EasyRobot::sendError(String MSG)
{
  Serial.println(MSG);
}

//--  Public Functions ----------------------------------------

// Initialize the robot, for unit pick either KMH or MMS
void EasyRobot::begin(unit speed_units, float stepsPerMillimeters, float speed, float Acceleration)
{

  leftMotor.setStepsPerMillimeter(stepsPerMillimeters);
  rightMotor.setStepsPerMillimeter(stepsPerMillimeters);

  if (speed_units == 1)
  {
    setSpeedInKMH(speed);
    setAccelerationInKMHH(Acceleration);
  }
  else
  {
    setSpeedInMMS(speed);
    setAccelerationInMMSS(Acceleration);
  }

  leftMotor.setCurrentPositionInMillimeters(0L);
  rightMotor.setCurrentPositionInMillimeters(0L);

  leftMotor.enableStepper();
  rightMotor.enableStepper();

  x_pos = 0;
  y_pos = 0;
  a_pos = 0;
  L_MOVE_DONE = true;
  R_MOVE_DONE = true;
  ROT_MOVE = false;
  STR_MOVE = false;
  previous_cord_LS = 0;
  previous_cord_RS = 0;
  current_cord_LS = 0;
  current_cord_RS = 0;
  target_cord_LS = 0;
  target_cord_RS = 0;
}
void EasyRobot::setUpPins(int leftMotorStepPin, int leftMotorDirPin, int leftMotorEnablePin, int rightMotorStepPin, int rightMotorDirPin, int rightMotorEnablePin)
{
  leftMotor.connectToPins(leftMotorStepPin, leftMotorDirPin, leftMotorEnablePin);
  rightMotor.connectToPins(rightMotorStepPin, rightMotorDirPin, rightMotorEnablePin);
}

// Overide the position stored by teh robot
void EasyRobot::setPosition(double angle)
{
  a_pos = angle;
}

// Overide the position stored by teh robot
void EasyRobot::setPosition(long xPos, long yPos)
{
  x_pos = xPos;
  y_pos = yPos;
}

// Overide the position stored by teh robot
void EasyRobot::setPosition(long xPos, long yPos, double angle)
{
  x_pos = xPos;
  y_pos = yPos;
  a_pos = angle;
}

// Get the current X coordinate
long EasyRobot::getXCoordinate() const
{
  return x_pos;
}

// Get the current Y coordinate
long EasyRobot::getYCoordinate() const
{
  return y_pos;
}

// Get the current a_pos (in radians)
double EasyRobot::getOrientation() const
{
  return a_pos;
}

// Move the robot directly to a target position (X, Y)
void EasyRobot::moveTo(long targetX, long targetY)
{

  long deltaX = targetX - x_pos;
  long deltaY = targetY - y_pos;

  setUpTurn(calc_orientation(deltaX, deltaY));
  while (!motionComplete())
  {
    processMovement();
  }
  // a_pos = calculateOrientation(deltaX, deltaY);

  long distance_diagonal = calc_diagonal_distance(deltaX, deltaY);

  target_cord_LS = distance_diagonal;
  target_cord_RS = distance_diagonal;
  STR_MOVE = true;
  leftMotor.setTargetPositionInMillimeters(target_cord_LS);
  rightMotor.setTargetPositionInMillimeters(target_cord_RS);

  // Update the position and a_pos
  // updatePosition(deltaX, deltaY, deltaOrientation);
}

bool EasyRobot::motionComplete()
{
  // if (leftMotor.motionComplete())
  // {
  //   target_cord_LS = 0;
  //   leftMotor.setCurrentPositionInMillimeters(0.00);
  //   leftMotor.setTargetPositionInMillimeters(0.00);
  // }
  // if (rightMotor.motionComplete())
  // {
  //   target_cord_RS = 0;
  //   rightMotor.setCurrentPositionInMillimeters(0.00);
  //   rightMotor.setTargetPositionInMillimeters(0.00);
  // }
  if (leftMotor.motionComplete() && rightMotor.motionComplete())
  {
    STR_MOVE = false;
    ROT_MOVE = false;
    load_move();
    return true;
  }
  else
  {
    return false;
  }
}

bool EasyRobot::processMovement()
{

  if (!motionComplete())
  {
    leftMotor.processMovement();
    rightMotor.processMovement();

    long delta_LS = 0;
    long delta_RS = 0;
    long delta_diag = 0;
    double delta_orientatoion = 0;
    long delatX = 0;
    long deltaY = 0;

    bool update_pos_flag1 = false;
    bool update_pos_flag2 = false;

    current_cord_LS = leftMotor.getCurrentPositionInMillimeters();
    current_cord_RS = rightMotor.getCurrentPositionInMillimeters();
    if ((current_cord_LS - previous_cord_LS) == 1)
    {
      previous_cord_LS = current_cord_LS;
      delta_LS = 1;
      update_pos_flag1 = true;
    }

    if ((current_cord_RS - previous_cord_RS) == 1)
    {
      previous_cord_RS = current_cord_RS;
      delta_RS = 1;
      update_pos_flag2 = true;
    }

    if (update_pos_flag1 && update_pos_flag2)
    {
      if (ROT_MOVE && !STR_MOVE)
      {
        delta_orientatoion = (delta_LS + (-delta_RS) / 2.00) / rotationConstant;
        // Serial.println("ROATE MOVE ACTIVE, chnage in A: " + (String) delta_orientatoion);
      }
      else if (!ROT_MOVE && STR_MOVE)
      {

        if (delta_LS != delta_RS)
        {
          delta_diag = ((abs(delta_LS) + abs(delta_RS)) / 2.00);
          Serial.println("ERROR---Stepper Values not the same---");
        }
        else
        {
          delta_diag = delta_LS;
        }
        Serial.println("CURRENT DIAG POS: " + (String)current_cord_LS);
        Serial.println("DELTA DIST: " + (String)delta_diag);
        Serial.println("Previous cord: " + (String)previous_cord_LS);

        delatX = calc_delta_X(delta_diag);
        deltaY = calc_delta_Y(delta_diag);
      }
      else if (ROT_MOVE && STR_MOVE)
      {
        Serial.println("ERROR---STRAIGHT MOVE AND ROTATE MOVE DETECTED SIMULTANEOUSLY");
        return 0;
      }
      else
      {
        Serial.println("ERROR---NO VALID MOVE DETECTED");
        return 0;
      }
      updatePosition(delatX, deltaY, delta_orientatoion);
    }
    return false;
  }
  return true;
}

// Move the robot forward
void EasyRobot::setupMoveForward(long distance)
{
  leftMotor.setTargetPositionInMillimeters(leftMotor.getCurrentPositionInMillimeters() + distance);
  rightMotor.setTargetPositionInMillimeters(rightMotor.getCurrentPositionInMillimeters() + distance);
}

void EasyRobot::setUpMove(long target_x, long target_y)
{
  leftMotor.setCurrentPositionInMillimeters(0);
  rightMotor.setCurrentPositionInMillimeters(0);

  long diagonal_distance = calc_diagonal_distance(target_x, target_y);

  leftMotor.setTargetPositionInMillimeters(diagonal_distance);
  rightMotor.setTargetPositionInMillimeters(diagonal_distance);
  STR_MOVE = true;
  ROT_MOVE = false;
}

// Turn the robot by a given angle (in radians)
void EasyRobot::setUpTurn(double TargetOrientation)
{
  leftMotor.setCurrentPositionInMillimeters(0);
  rightMotor.setCurrentPositionInMillimeters(0);
  float angle = TargetOrientation - a_pos;
  if (angle > PI)
  {
    angle = angle - (2 * PI);
  }
  else if (angle < -PI)
  {
    angle = angle + (2 * PI);
  }
  float distance = angle * rotationConstant;
  leftMotor.setTargetPositionInMillimeters(distance);
  rightMotor.setTargetPositionInMillimeters(-distance);
  ROT_MOVE = true;
  STR_MOVE = false;
  // if (distance != 0)
  // {
  //   ROT_MOVE = true;
  // }
}

void EasyRobot::setUpTurnDeg(double angle)
{
  setUpTurn(2 * PI * (angle / 360));
}

// Stop the robot // This is a blocking function while to robot decelerates to come to a stop
void EasyRobot::stop()
{
  // leftMotor.setupStop();
  // rightMotor.setupStop();
  while (1)
  {
    if (processMovement())
    {
      leftMotor.setCurrentPositionInSteps(0);
      rightMotor.setCurrentPositionInSteps(0);
    }
  }
}

float EasyRobot::toDeg(double rad)
{
  return (int)(rad * (180.0 / PI));
}
float EasyRobot::toRad(double deg)
{
  return deg * (PI / 180.0);
}

bool EasyRobot::add_move(long targetX, long targetY)
{
  move temp(targetX, targetY);
  if (!move_q.isFull())
  {
    move_q.enqueue(temp);
    return true;
  }
  else
  {
    sendError("MOVE QUEUE IS FULL---");
    return false;
  }
}

bool EasyRobot::add_move(double targetA)
{
  move temp(targetA);
  if (!move_q.isFull())
  {
    move_q.enqueue(temp);
    return true;
  }
  else
  {
    sendError("MOVE QUEUE IS FULL---");
    return false;
  }
}
