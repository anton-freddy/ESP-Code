#include <EasyRobot.h>

ArduinoQueue<move> move_q(500);

//--  Constructor
EasyRobot::EasyRobot(float wheel_circumfrence, float wheel_distance, int MICRO_STEP, int STEPPER_STEP_COUNT, int GEAR_RATIO)
{
  steps_per_rev = MICRO_STEP * STEPPER_STEP_COUNT * GEAR_RATIO;
  wheel_circumfrence_mm = wheel_circumfrence;
  wheel_distance_mm = wheel_distance;
  steps_per_mm = steps_per_rev / wheel_circumfrence_mm;
  micro_step = MICRO_STEP;
  stepper_steps_per_rev = STEPPER_STEP_COUNT;
  gear_ratio = GEAR_RATIO;
}

//--  Stepper Movement -----------------------------------------

void EasyRobot::update_stepper_DIR_pin()
{
  switch (L_direction)
  {
  case 1:
    digitalWrite(L_D_pin, LOW);
    break;

  case -1:
    digitalWrite(L_D_pin, HIGH);
    break;

  default:
    send_ERROR(LEFT_STEPPER, 0x00);
    break;
  }
  switch (R_direction)
  {
  case 1:
    digitalWrite(R_D_pin, LOW);
    break;

  case -1:
    digitalWrite(R_D_pin, HIGH);
    break;

  default:
    send_ERROR(RIGHT_STEPPER, 0x00);
    break;
  }
}

bool EasyRobot::L_stepper_target_reached()
{
  switch (L_direction)
  {
  case 1:
    if (L_Current_POS_CM < L_Target_POS_CM)
    {
      return false;
    }
    else if (L_Current_POS_CM > L_Target_POS_CM)
    {
      L_direction = L_direction * -1;
      return false;
    }
    else if (L_Current_POS_CM == L_Target_POS_CM)
    {
      return true;
    }
    else
    {
      return false;
    }
    break;

  case -1:
    if (L_Current_POS_CM > L_Target_POS_CM)
    {
      return false;
    }
    else if (L_Current_POS_CM < L_Target_POS_CM)
    {
      L_direction = L_direction * -1;
      return false;
    }
    else if (L_Current_POS_CM == L_Target_POS_CM)
    {
      return true;
    }
    else
    {
      return false;
    }
    break;

  default:
    send_ERROR(LEFT_STEPPER, 0x01);
    return false;
    break;
  }
}
bool EasyRobot::R_stepper_target_reached()
{
  switch (R_direction)
  {
  case 1:
    if (R_Current_POS_CM < R_Target_POS_CM)
    {
      return false;
    }
    else if (R_Current_POS_CM > R_Target_POS_CM)
    {
      R_direction = -1 * R_direction;
      return false;
    }
    else if (R_Current_POS_CM == R_Target_POS_CM)
    {
      return true;
    }
    else
    {

      return false;
    }
    break;

  case -1:
    if (R_Current_POS_CM > R_Target_POS_CM)
    {
      return false;
    }
    else if (R_Current_POS_CM < R_Target_POS_CM)
    {
      R_direction = -1 * R_direction;
      return false;
    }
    else if (R_Current_POS_CM == R_Target_POS_CM)
    {
      return true;
    }
    else
    {

      return false;
    }
    break;

  default:
    send_ERROR(RIGHT_STEPPER, 0x01);
    return false;
    break;
  }
}

void EasyRobot::moveSteppers()
{
  update_stepper_DIR_pin();

  L_step_time = 10000000 / L_Speed_SPS;
  R_step_time = 10000000 / R_Speed_SPS;

  if (L_STEP_MOVING)
  {
    enableStepper(left);
    long current_time = micros();
    // long current_time = millis();
    if ((current_time - L_previous_time) >= L_step_time)
    {
      L_previous_time = current_time;
      digitalWrite(L_S_pin, HIGH);
    }
    digitalWrite(L_S_pin, LOW);
  }
  else
  {
    disableStepper(left);
  }

  if (R_STEP_MOVING)
  {
    enableStepper(right);
    long current_time = micros();
    // long current_time = millis();
    if ((current_time - R_previous_time) >= R_step_time)
    {
      R_previous_time = current_time;
      digitalWrite(R_S_pin, HIGH);
    }
    digitalWrite(R_S_pin, LOW);
  }
  else
  {
    disableStepper(right);
  }
}

void EasyRobot::L_setSpeed_SPS(long SPS)
{
  L_Speed_SPS = SPS;
}
void EasyRobot::R_setSpeed_SPS(long SPS)
{
  R_Speed_SPS = SPS;
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

float EasyRobot::L_getCurrentPos_CM(void)
{
  return L_Current_POS_CM; // / steps_per_mm;
}
float EasyRobot::L_getCurrentSpeed_MMS(void)
{
  return L_Speed_SPS / steps_per_mm;
}
float EasyRobot::R_getCurrentPos_CM(void)
{
  return R_Current_POS_CM; // / steps_per_mm;
}
float EasyRobot::R_getCurrentSpeed_MMS(void)
{
  return R_Speed_SPS / steps_per_mm;
}

void EasyRobot::L_setTarget_POS(float Target_MM)
{
  L_Target_POS_CM = Target_MM;
  if (Target_MM < 0)
  {
    L_direction = -1;
  }
  else
  {
    L_direction = 1;
  }
}

void EasyRobot::R_setTarget_POS(float Target_MM)
{
  R_Target_POS_CM = Target_MM;
  if (Target_MM < 0)
  {
    R_direction = -1;
  }
  else
  {
    R_direction = 1;
  }
}

//--  Private Functions ----------------------------------------

// Updates the position of the robot, values passed should be delta values, NOT target values
void EasyRobot::updatePosition(float deltaX, float deltaY, float deltaOrientation)
{
  updatePosition(deltaX, deltaY);
  updatePosition(deltaOrientation);
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

void EasyRobot::updatePose()
{


  float L_ENC_READ = getEncoderAngle(left);
  float R_ENC_READ = getEncoderAngle(right);
  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - pose_previousTime) / 1000.0; // Convert to seconds

  // Read encoder values

  float L_delta_theta = L_ENC_READ - L_ENC_PREVIOUS;
  float R_delta_theta = R_ENC_READ - R_ENC_PREVIOUS;
  L_ENC_PREVIOUS = L_ENC_READ;
  R_ENC_PREVIOUS = R_ENC_READ;
  // Calculate the change in angle accounting for overflow
  if (L_delta_theta > 180)
  {
    L_delta_theta -= 360.0; // Account for counter-clockwise movement
  }
  else if (L_delta_theta < -180)
  {
    L_delta_theta += 360.0; // Account for clockwise movement
  }

  if (R_delta_theta > 180)
  {
    R_delta_theta -= 360.0; // Account for counter-clockwise movement
  }
  else if (R_delta_theta < -180)
  {
    R_delta_theta += 360.0; // Account for clockwise movement
  }

  // Calculate the linear distance traveled
  float L_delta_CM = (L_delta_theta / 360.0) * (wheel_circumfrence_mm / 10) * gear_ratio;
  float R_delta_CM = (R_delta_theta / 360.0) * (wheel_circumfrence_mm / 10) * gear_ratio;

  float leftDist = (L_delta_theta / 360.0) * (wheel_circumfrence_mm / 10) * gear_ratio;  // Function to get left encoder distance change in cm
  float rightDist = (R_delta_theta / 360.0) * (wheel_circumfrence_mm / 10) * gear_ratio; // Function to get right encoder distance change in cm

  // Calculate wheel velocities
  float leftVel = (leftDist - L_prev_dist) / elapsedTime;
  float rightVel = (rightDist - R_prev_dist) / elapsedTime;

  // Calculate robot linear and angular velocities
  float linearVel = (leftVel + rightVel) / 2.0;
  float angularVel = (rightVel - leftVel) / (wheel_distance_mm / 10);

  // Update robot pose
  updatePosition(angularVel * elapsedTime);
  updatePosition((linearVel * cos(getOrientation()) * elapsedTime),(linearVel * sin(getOrientation()) * elapsedTime));

  // Store current values for next iteration
  L_prev_dist = leftDist;
  R_prev_dist = rightDist;
  pose_previousTime = currentTime;
  Serial.println("delta Theta: " + (String)(angularVel * elapsedTime));
    Serial.println("delta X: " + (String)(linearVel * cos(getOrientation()) * elapsedTime));
      Serial.println("delta Y: " + (String)(linearVel * sin(getOrientation()) * elapsedTime));

  Serial.println("Linear Vel: " + (String)linearVel);
  Serial.println("Angular Vel: " + (String)angularVel);
  Serial.println("Left Distance: " + (String)leftDist);
  Serial.println("Right Distance: " + (String)rightDist);
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
  L_setSpeed_MMPS(speed);
  R_setSpeed_MMPS(speed);
}

// Sets the speed of the robot in KM/h, !!NOTE!! you must set the steps per millimeter before setting the speed
void EasyRobot::setAccelerationInKMHH(float speed)
{
  setAccelerationInMMSS(speed * 0.2778);
}

// Sets the speed of the robot in mm/s, !!NOTE!! you must set the steps per millimeter before setting the speed
void EasyRobot::setAccelerationInMMSS(float speed)
{

  // leftMotor.setAccelerationInMillimetersPerSecondPerSecond(speed);
  // rightMotor.setAccelerationInMillimetersPerSecondPerSecond(speed);
  setAcceleration_SPSPS(steps_per_mm * speed);
}

//--  Public Functions ----------------------------------------

// Initialize the robot, for unit pick either KMH or MMS
void EasyRobot::begin(unit speed_units, float speed, float Acceleration)
{

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
  L_Current_POS_CM = 0;
  R_Current_POS_CM = 0;
  L_Target_POS_CM = 0;
  R_Target_POS_CM = 0;

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

  Serial.println("ROOMBA SETUP COMPLETE");
}
void EasyRobot::setUpMotors(byte leftMotorStepPin, byte leftMotorDirPin, byte leftMotorEnablePin, byte rightMotorStepPin, byte rightMotorDirPin, byte rightMotorEnablePin)
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

  Serial.println("MOTOR SETUP COMPLETE");
}

void EasyRobot::setUpEncoders(byte L_ENC_SDA, byte L_ENC_SCL, byte R_ENC_SDA, byte R_ENC_SCL)
{
  Wire1.begin(L_ENC_SDA, L_ENC_SCL);
  Wire.begin(R_ENC_SDA, R_ENC_SCL);

  if (L_ENCODER.detectMagnet() == 0)
  {
    send_ERROR(LEFT_ENCODER, 0x00);
  }
  else if (L_ENCODER.detectMagnet() == 1)
  {
    Serial.println("L ENCODER MAGNET FOUND");
  }

  if (R_ENCODER.detectMagnet() == 0)
  {
    send_ERROR(RIGHT_ENCODER, 0x00);
  }
  else if (R_ENCODER.detectMagnet() == 1)
  {
    Serial.println("R ENCODER MAGNET FOUND");
  }
  resetEncoders(both);

  Serial.println("ENCODER SETUP COMPLETE");
}

void EasyRobot::resetEncoders(motor selector)
{

  switch (selector)
  {
  case left:
    L_ENC_PREVIOUS = getEncoderAngle(left);
    break;

  case right:
    R_ENC_PREVIOUS = getEncoderAngle(right);
    break;

  default:
    L_ENC_PREVIOUS = getEncoderAngle(left);
    R_ENC_PREVIOUS = getEncoderAngle(right);
    break;
  }
}

float EasyRobot::getEncoderAngle(motor identifier)
{
  float x, in_min, in_max, out_min, out_max;
  in_min = 0;
  in_max = 4095;
  out_min = 0;
  out_max = 360;
  float newAngle = -1;
  switch (identifier)
  {
  case left:
    newAngle = L_ENCODER.getRawAngle();
    break;

  case right:
    newAngle = R_ENCODER.getRawAngle();
    break;

  default:
    send_ERROR(ENCODERS, 0x02);
    return 0;
  }
  x = newAngle;
  return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);

  /* Raw data reports 0 - 4095 segments, which is 0.087890625 of a degree */
  // return (float)map_f(newAngle,0.0,4095.0,0.0,360.0);//(newAngle * 0.087890625)
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

  L_Target_POS_CM = distance_diagonal;
  R_Target_POS_CM = distance_diagonal;
  STR_MOVE = true;
  L_setTarget_POS(target_cord_LS);
  R_setTarget_POS(target_cord_RS);

  // Update the position and a_pos
  // updatePosition(deltaX, deltaY, deltaOrientation);
}

bool EasyRobot::motionComplete()
{

  if ((L_stepper_target_reached()) && (R_stepper_target_reached()))
  {
    STR_MOVE = false;
    ROT_MOVE = false;
    L_STEP_MOVING = false;
    L_STEP_MOVING = false;
    // load_move();
    return true;
  }
  else
  {
    if (!L_stepper_target_reached())
    {
      L_STEP_MOVING = true;
    }
    else
    {
      L_STEP_MOVING = false;
    }
    if (!R_stepper_target_reached())
    {
      R_STEP_MOVING = true;
    }
    else
    {
      R_STEP_MOVING = false;
    }

    return false;
  }
}

long previous___Millis = 0;

bool EasyRobot::processMovement()
{
  //UpdatePosFromEncoders(10);
  moveSteppers();
  long current___Millis = millis();
  if (current___Millis - previous___Millis >= 300)
  {
    //Serial.println("X: " + (String)getXCoordinate() + " Y: " + (String)getYCoordinate() + " A: " + (String)getOrientation());
  }
  if (!L_stepper_target_reached())
  {
    L_STEP_MOVING = true;
  }
  else
  {
    L_STEP_MOVING = false;
  }

  if (!R_stepper_target_reached())
  {
    R_STEP_MOVING = true;
  }
  else
  {
    R_STEP_MOVING = false;
  }

  return true;
}

void EasyRobot::UpdatePosFromEncoders(long refresh_rate)
{
  encoder_current_millis = millis();
  if (encoder_current_millis - encoder_previous_millis >= refresh_rate)
  {
    encoder_previous_millis = encoder_current_millis;
    float L_ENC_READ = getEncoderAngle(left);
    float R_ENC_READ = getEncoderAngle(right);
    float L_delta_theta = L_ENC_READ - L_ENC_PREVIOUS;
    float R_delta_theta = R_ENC_READ - R_ENC_PREVIOUS;
    L_ENC_PREVIOUS = L_ENC_READ;
    R_ENC_PREVIOUS = R_ENC_READ;
    // Calculate the change in angle accounting for overflow
    if (L_delta_theta > 180)
    {
      L_delta_theta -= 360.0; // Account for counter-clockwise movement
    }
    else if (L_delta_theta < -180)
    {
      L_delta_theta += 360.0; // Account for clockwise movement
    }

    if (R_delta_theta > 180)
    {
      R_delta_theta -= 360.0; // Account for counter-clockwise movement
    }
    else if (R_delta_theta < -180)
    {
      R_delta_theta += 360.0; // Account for clockwise movement
    }

    // Calculate the linear distance traveled
    float L_delta_CM = (L_delta_theta / 360.0) * (wheel_circumfrence_mm / 10) * gear_ratio;
    float R_delta_CM = (R_delta_theta / 360.0) * (wheel_circumfrence_mm / 10) * gear_ratio;

    float delta_X = 0;
    float delta_Y = 0;
    float delta_theta = 0;
    if (STR_MOVE && !ROT_MOVE)
    {
      delta_X = calc_delta_X((L_delta_CM + R_delta_CM) / 2);
      delta_Y = calc_delta_Y((L_delta_CM + R_delta_CM) / 2);
    }
    else if (!STR_MOVE && ROT_MOVE)
    {
      wheel_distance_mm;
    }
    L_Current_POS_CM += L_delta_CM;
    R_Current_POS_CM += R_delta_CM;
  }
}

// Move the robot forward
void EasyRobot::setupMoveForward(float distance)
{
  L_Current_POS_CM = 0;
  R_Current_POS_CM = 0;

  L_setTarget_POS(distance);
  R_setTarget_POS(distance);

  STR_MOVE = true;
  ROT_MOVE = false;
}

void EasyRobot::setUpMove(float target_x, float target_y)
{
  L_Current_POS_CM = 0;
  R_Current_POS_CM = 0;

  float diagonal_distance = calc_diagonal_distance(target_x, target_y);

  L_setTarget_POS(diagonal_distance);
  R_setTarget_POS(diagonal_distance);
  STR_MOVE = true;
  ROT_MOVE = false;
}

// Turn the robot by a given angle (in radians)
void EasyRobot::setUpTurn(float TargetOrientation)
{
  L_Current_POS_CM = 0;
  R_Current_POS_CM = 0;
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
  L_setTarget_POS(distance);
  R_setTarget_POS(-distance);
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
  L_STEP_MOVING = false;
  R_STEP_MOVING = false;
}

void EasyRobot::resume()
{
  L_STEP_MOVING = true;
  R_STEP_MOVING = true;
}

void EasyRobot::enableStepper(motor select)
{
  if (select == left)
  {
    digitalWrite(L_E_pin, LOW);
  }
  if (select == right)
  {
    digitalWrite(R_E_pin, LOW);
  }
  if (select == both)
  {
    digitalWrite(R_E_pin, LOW);
    digitalWrite(L_E_pin, LOW);
  }
}
void EasyRobot::disableStepper(motor select)
{
  if (select == left)
  {
    digitalWrite(L_E_pin, HIGH);
  }
  if (select == right)
  {
    digitalWrite(R_E_pin, HIGH);
  }
  if (select == both)
  {
    digitalWrite(R_E_pin, HIGH);
    digitalWrite(L_E_pin, HIGH);
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
    send_ERROR(MOVEMENT, 0x00);
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
    send_ERROR(MOVEMENT, 0x00);
    return false;
  }
}
