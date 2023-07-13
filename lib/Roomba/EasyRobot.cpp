#include <EasyRobot.h>

ArduinoQueue<move> move_q(500);

//--  Constructor
EasyRobot::EasyRobot(int steps_per_revolution, float wheel_circumfrence, float wheel_distance)
{
  steps_per_rev = steps_per_revolution;
  wheel_circumfrence_mm = wheel_circumfrence;
  wheel_distance_mm = wheel_distance;
  steps_per_mm = steps_per_rev / wheel_circumfrence_mm;
}

//--  Stepper Movement -----------------------------------------

bool EasyRobot::moveSteppers()
{
  L_step_time = 1000 / L_TargetSpeed;
  R_step_time = 1000 / R_TargetSpeed;
  if (abs(L_CurrentSteps) < abs(L_TargetSteps))
  {
    long current_time = millis();
    if ((current_time - L_previous_time) >= L_step_time)
    {
      L_previous_time = current_time;
      digitalWrite(L_S_pin, HIGH);
      if (L_direction == 1)
      {
        L_CurrentSteps++;
      }
      else if (L_direction == -1)
      {
        L_CurrentSteps--;
      }
    }
    digitalWrite(L_S_pin, LOW);
  }

  if (abs(R_CurrentSteps) < abs(R_TargetSteps))
  {
    long current_time = millis();
    if ((current_time - R_previous_time) >= R_step_time)
    {
      R_previous_time = current_time;
      digitalWrite(R_S_pin, HIGH);
      if (R_direction == 1)
      {
        R_CurrentSteps++;
      }
      else if (R_direction == -1)
      {
        R_CurrentSteps--;
      }
    }
    digitalWrite(R_S_pin, LOW);
  }
  if (L_CurrentSteps == L_TargetSteps && R_CurrentSteps == R_TargetSteps)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void EasyRobot::L_setSpeed_SPS(long SPS)
{
  L_TargetSpeed = SPS;
  L_CurrentSpeed = L_TargetSpeed;
}
void EasyRobot::R_setSpeed_SPS(long SPS)
{
  R_TargetSpeed = SPS;
  R_CurrentSpeed = R_TargetSpeed;
}
void EasyRobot::L_setSpeed_MMPS(float MMPS)
{
  L_setSpeed_SPS(MMPS * steps_per_mm);
}
void EasyRobot::R_setSpeed_MMPS(float MMPS)
{
  R_setSpeed_SPS(MMPS * steps_per_mm);
}
void EasyRobot::setAcceleration_SPSPS(long SPSPS)
{
}

float EasyRobot::L_getCurrentPos_MM(void)
{
  return L_CurrentSteps / steps_per_mm;
}
float EasyRobot::L_getCurrentSpeed_MMS(void)
{
  return L_CurrentSpeed / steps_per_mm;
}
float EasyRobot::R_getCurrentPos_MM(void)
{
  return R_CurrentSteps / steps_per_mm;
}
float EasyRobot::R_getCurrentSpeed_MMS(void)
{
  return R_CurrentSpeed / steps_per_mm;
}

void EasyRobot::L_setTarget_MM(float Target_MM)
{
  L_setTarget_STEP(Target_MM * steps_per_mm);
}
void EasyRobot::L_setTarget_STEP(long Target_STEP)
{
  L_TargetSteps = Target_STEP;
  if (L_TargetSteps < L_CurrentSteps)
  {
    L_direction = -1;
    digitalWrite(L_D_pin, LOW);
  }
  else
  {
    L_direction = 1;
    digitalWrite(L_D_pin, HIGH);
  }
}
void EasyRobot::R_setTarget_MM(float Target_MM)
{
  R_setTarget_STEP(Target_MM * steps_per_mm);
}
void EasyRobot::R_setTarget_STEP(long Target_STEP)
{
  R_TargetSteps = Target_STEP;
  if (R_TargetSteps < R_CurrentSteps)
  {
    R_direction = -1;
    digitalWrite(R_D_pin, LOW);
  }
  else
  {
    R_direction = 1;
    digitalWrite(R_D_pin, HIGH);
  }
}
//--  Private Functions ----------------------------------------

// Updates the position of the robot, values passed should be delta values, NOT target values
void EasyRobot::updatePosition(float deltaX, float deltaY, float deltaOrientation)
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
void EasyRobot::updatePosition(float deltaX, float deltaY)
{
  x_pos += deltaX;
  y_pos += deltaY;
}

// Updates the position of the robot, values passed should be delta values, NOT target values
void EasyRobot::updatePosition(float deltaOrientation)
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
float EasyRobot::calc_orientation(float target_x, float target_y)
{
  float deltaX = target_x - x_pos;
  float deltaY = target_y - y_pos;
  // return atan2f(deltaY, deltaX);
  float target_orientation = 0;
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
      target_orientation = atan(deltaX / deltaY);
    }
    else if (deltaY < 0)
    {
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
      target_orientation = PI + atan(abs(deltaX / deltaY));
    }
    else if (deltaY > 0)
    {
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
float EasyRobot::calc_delta_Y(float straight_line_dist)
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
float EasyRobot::calc_delta_X(float straight_line_dist)
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
float EasyRobot::calc_diagonal_distance(float target_x, float target_y)
{
  float deltaX = target_x - x_pos;
  float deltaY = target_y - y_pos;
  float distance_diagonal;

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
  L_TargetSpeed = speed * steps_per_mm;
  R_TargetSpeed = speed * steps_per_mm;
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
void EasyRobot::begin(unit speed_units, float speed, float Acceleration)
{

  if (speed_units == 1)
  {
    setSpeedInKMH(speed);
    // setAccelerationInKMHH(Acceleration);
  }
  else
  {
    setSpeedInMMS(speed);
    // setAccelerationInMMSS(Acceleration);
  }
  L_CurrentSteps = 0;
  R_CurrentSteps = 0;
  L_TargetSteps = 0;
  R_TargetSteps = 0;


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
void EasyRobot::setUpPins(byte leftMotorStepPin, byte leftMotorDirPin, byte leftMotorEnablePin, byte rightMotorStepPin, byte rightMotorDirPin, byte rightMotorEnablePin)
{
  L_S_pin = leftMotorStepPin;
  L_D_pin = leftMotorDirPin;
  L_E_pin = leftMotorEnablePin;
  pinMode(L_S_pin, OUTPUT);
  pinMode(L_D_pin, OUTPUT);
  pinMode(L_E_pin, OUTPUT);
  digitalWrite(L_S_pin, LOW);
  digitalWrite(L_D_pin, LOW);
  digitalWrite(L_E_pin, LOW);

  R_S_pin = rightMotorStepPin;
  R_D_pin = rightMotorDirPin;
  R_E_pin = rightMotorEnablePin;
  pinMode(R_S_pin, OUTPUT);
  pinMode(R_D_pin, OUTPUT);
  pinMode(R_E_pin, OUTPUT);
  digitalWrite(R_S_pin, LOW);
  digitalWrite(R_D_pin, LOW);
  digitalWrite(R_E_pin, LOW);
}

// Overide the position stored by teh robot
void EasyRobot::setPosition(float angle)
{
  a_pos = angle;
}

// Overide the position stored by teh robot
void EasyRobot::setPosition(float xPos, float yPos)
{
  x_pos = xPos;
  y_pos = yPos;
}

// Overide the position stored by teh robot
void EasyRobot::setPosition(float xPos, float yPos, float angle)
{
  x_pos = xPos;
  y_pos = yPos;
  a_pos = angle;
}

// Get the current X coordinate
float EasyRobot::getXCoordinate() const
{
  return x_pos;
}

// Get the current Y coordinate
float EasyRobot::getYCoordinate() const
{
  return y_pos;
}

// Get the current a_pos (in radians)
float EasyRobot::getOrientation() const
{
  return a_pos;
}

// Move the robot directly to a target position (X, Y)
void EasyRobot::moveTo(float targetX, float targetY)
{

  float deltaX = targetX - x_pos;
  float deltaY = targetY - y_pos;

  setUpTurn(calc_orientation(deltaX, deltaY));
  while (!motionComplete())
  {
    processMovement();
  }
  // a_pos = calculateOrientation(deltaX, deltaY);

  float distance_diagonal = calc_diagonal_distance(deltaX, deltaY);

  target_cord_LS = distance_diagonal;
  target_cord_RS = distance_diagonal;
  STR_MOVE = true;
  L_setTarget_MM(target_cord_LS);
  R_setTarget_MM(target_cord_RS);

  // Update the position and a_pos
  // updatePosition(deltaX, deltaY, deltaOrientation);
}

bool EasyRobot::motionComplete()
{

  if ((L_TargetSteps == L_CurrentSteps) && (R_TargetSteps == R_CurrentSteps))
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
    moveSteppers();

    float delta_LS = 0;
    float delta_RS = 0;
    float delta_diag = 0;
    float delta_orientatoion = 0;
    float delatX = 0;
    float deltaY = 0;

    bool update_pos_flag1 = false;
    bool update_pos_flag2 = false;

    current_cord_LS = L_getCurrentPos_MM();
    current_cord_RS = R_getCurrentPos_MM();
    if ((current_cord_LS - previous_cord_LS) >= 10)
    {
      delta_LS = current_cord_LS - previous_cord_LS;
      previous_cord_LS = current_cord_LS;
      update_pos_flag1 = true;
    }

    if ((current_cord_RS - previous_cord_RS) >= 10)
    {
      delta_RS = current_cord_RS - previous_cord_RS;
      previous_cord_RS = current_cord_RS;
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
        // Serial.println("CURRENT DIAG POS: " + (String)current_cord_LS);
        // Serial.println("DELTA DIST: " + (String)delta_diag);
        // Serial.println("Previous cord: " + (String)previous_cord_LS);

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
void EasyRobot::setupMoveForward(float distance)
{
  L_setTarget_MM(distance);
  R_setTarget_MM(distance);
}

void EasyRobot::setUpMove(float target_x, float target_y)
{
  L_CurrentSteps = 0;
  R_CurrentSteps = 0;

  float diagonal_distance = calc_diagonal_distance(target_x, target_y);

  L_setTarget_MM(diagonal_distance);
  R_setTarget_MM(diagonal_distance);
  STR_MOVE = true;
  ROT_MOVE = false;
}

// Turn the robot by a given angle (in radians)
void EasyRobot::setUpTurn(float TargetOrientation)
{
  L_CurrentSteps = 0;
  R_CurrentSteps = 0;
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
  L_setTarget_MM(distance);
  R_setTarget_MM(-distance);
  ROT_MOVE = true;
  STR_MOVE = false;
  // if (distance != 0)
  // {
  //   ROT_MOVE = true;
  // }
}

void EasyRobot::setUpTurnDeg(float angle)
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

float EasyRobot::toDeg(float rad)
{
  return (int)(rad * (180.0 / PI));
}
float EasyRobot::toRad(float deg)
{
  return deg * (PI / 180.0);
}

bool EasyRobot::add_move(float targetX, float targetY)
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

bool EasyRobot::add_move(float targetA)
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
