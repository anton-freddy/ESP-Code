#include <EasyRobot.h>

const float rotationConstant = 171.931; // 158.3;//160.49877; // Must be adjusted based on wheel distance and mounting points

void EasyRobot::updatePosition(float deltaX, float deltaY, float deltaOrientation)
{
  xCoordinate += deltaX;
  yCoordinate += deltaY;
  orientation += deltaOrientation;
}

void EasyRobot::updatePosition(float deltaX, float deltaY)
{
  xCoordinate += deltaX;
  yCoordinate += deltaY;
}

void EasyRobot::updatePosition(float deltaOrientation)
{
  orientation += deltaOrientation;
}

// Constructor
EasyRobot::EasyRobot()
{
}

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

  xCoordinate = 0;
  yCoordinate = 0;
  orientation = 0;
  L_MOVE_DONE = true;
  R_MOVE_DONE = true;
  ROTATE_MOVE = false;
}

void EasyRobot::setPosition(float angle)
{
  orientation = angle;
}
void EasyRobot::setPosition(float xPos, float yPos)
{
  xCoordinate = xPos;
  yCoordinate = yPos;
}
void EasyRobot::setPosition(float xPos, float yPos, float angle)
{
  xCoordinate = xPos;
  yCoordinate = yPos;
  orientation = angle;
}

void EasyRobot::setUpPins(int leftMotorStepPin, int leftMotorDirPin, int leftMotorEnablePin, int rightMotorStepPin, int rightMotorDirPin, int rightMotorEnablePin)
{
  leftMotor.connectToPins(leftMotorStepPin, leftMotorDirPin, leftMotorEnablePin);
  rightMotor.connectToPins(rightMotorStepPin, rightMotorDirPin, rightMotorEnablePin);
}

// Get the current X coordinate
float EasyRobot::getXCoordinate() const
{
  return xCoordinate;
}

// Get the current Y coordinate
float EasyRobot::getYCoordinate() const
{
  return yCoordinate;
}

// Get the current orientation (in radians)
float EasyRobot::getOrientation() const
{
  return orientation;
}

// Returns the angle by which the robot needs to rotate based on the change in X and Y and the previous orientation
float EasyRobot::calculateOrientation(float deltaX, float deltaY)
{
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
      // Positive X, Positive Y - Quadrant 1
      // target_orientation = (atan(deltaX / deltaY));
      Serial.println("QUADRANT 1");
      target_orientation = atan(deltaX/deltaY);
    }
    else if (deltaY < 0)
    {
      Serial.println("QUADRANT 2");
      target_orientation = 0.5*PI + atan(abs(deltaY/deltaX));
    }
    else
    {
      Serial.println("ERROR X is positive but Y is not matched");
    }
  }
  else if (deltaX < 0 )
  {
    if(deltaY < 0){
      // Negative X, Negative Y - Quadrant 3
    Serial.println("QUADRANT 3");
    target_orientation = PI + atan(abs(deltaX/deltaY));
    } else if(deltaY > 0){
      // Negative X, Positive Y - Quadrant 4
    Serial.println("QUADRANT 4");
    target_orientation = 1.5*PI + atan(abs(deltaY/deltaX));
    } else {
      Serial.println("ERROR X is negative but Y is not matched");
    }
  }
  else
  {
    Serial.println((String)deltaX);
    Serial.println((String)deltaY);
    Serial.println("ERROR NO SUITABLE ORIENTATION FOUND");
    return 7777.7777777;
  }

  if (target_orientation > 2*PI)
  {
    Serial.println("ORIENTATION OUT OF BOUNDS, ABOVE 2PI");
    return 7777.7777777;
  }
  else if (target_orientation < 0)
  {
    Serial.println("ORIENTATION OUT OF BOUNDS, below 0");
    return 7777.7777777;
  }
  else
  {
    if(target_orientation == 2*PI){
      target_orientation = 0;
    }
    return target_orientation;
  }
}
// Move the robot directly to a target position (X, Y)
void EasyRobot::moveTo(float targetX, float targetY)
{
  float deltaX = targetX - xCoordinate;
  float deltaY = targetY - yCoordinate;

  // setUpTurn(calculateOrientation(deltaX,deltaY));
  // while (!motionComplete())
  // {
  //   processMovement();
  // }
  orientation = calculateOrientation(deltaX, deltaY);
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
    distance_diagonal = sqrt(powf(deltaX,2) + powf(deltaY, 2));//abs(deltaX / sin(getOrientation())); // sqrt(pow(deltaX, 2) + pow(deltaY, 2)); // Need to add signed to quadrant 3 & 4
  }

  // Move the robot using absolute position
  leftMotor.setCurrentPositionInMillimeters(0L);
  rightMotor.setCurrentPositionInMillimeters(0L);
  leftMotor.setTargetPositionInMillimeters(distance_diagonal);
  rightMotor.setTargetPositionInMillimeters(distance_diagonal);
  nextMoveDiagDistance = distance_diagonal;

  // Update the position and orientation
  // updatePosition(deltaX, deltaY, deltaOrientation);
}

bool EasyRobot::motionComplete()
{

  if (leftMotor.motionComplete() && rightMotor.motionComplete())
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool EasyRobot::processMovement()
{
  current_cord_LS = leftMotor.getCurrentPositionInMillimeters();
  current_cord_RS = rightMotor.getCurrentPositionInMillimeters();
  previous_cord_LS = current_cord_LS;
  previous_cord_RS = current_cord_RS;

  if (!motionComplete())
  {
    leftMotor.processMovement();
    rightMotor.processMovement();
    current_cord_LS = leftMotor.getCurrentPositionInMillimeters();
    current_cord_RS = rightMotor.getCurrentPositionInMillimeters();
  }

  float delta_LS = current_cord_LS - previous_cord_LS;
  float delta_RS = current_cord_RS - previous_cord_RS;
  float delta_diag = 0;
  float delta_orientatoion = 0;
  float delatX = 0;
  float deltaY = 0;

  if (ROTATE_MOVE)
  {
    delta_orientatoion = delta_LS / rotationConstant;
    //Serial.println("ROATE MOVE ACTIVE, chnage in A: " + (String) delta_orientatoion);
  }
  else
  {
    
    if (delta_LS != delta_RS)
    {
      delta_diag = ((delta_LS + delta_RS) / 2.00);
    }
    else
    {
      delta_diag = delta_LS;
    }

    delatX = calc_delta_X(delta_diag);
    deltaY = calc_delta_Y(delta_diag);
    
  }
  updatePosition(delatX, deltaY, delta_orientatoion);

  if (motionComplete())
  {

    leftMotor.setTargetPositionInMillimeters(0L);
    rightMotor.setTargetPositionInMillimeters(0L);
    leftMotor.setCurrentPositionInMillimeters(0L);
    rightMotor.setCurrentPositionInMillimeters(0L);
    ROTATE_MOVE = false;
    return true;
  }
  else
  {
    return false;
  }
}

float EasyRobot::calc_delta_Y(float straight_line_dist)
{
   if(orientation == 0){
    return 0; // No change in X
  } else if (orientation == (0.5*PI)) {
    return straight_line_dist; // No change in Y
  } else if (orientation == PI){
    return 0; // No chnage along X axis
  } else if (orientation == 1.5*PI){
    return -straight_line_dist; // No change in Y
  } else if (orientation > 0 && orientation < (0.5*PI)){
    return straight_line_dist * sinf(orientation);
  } else if (orientation > (0.5*PI) && orientation < PI){
    return straight_line_dist * sinf(orientation-(0.5*PI));
  } else if (orientation > PI && orientation < (1.5*PI)){
    return -(straight_line_dist * sinf(orientation-PI));
  } else if (orientation > (1.5*PI) && orientation < (2*PI)){
    return -(straight_line_dist * sinf(orientation-(1.5*PI)));
  } else{
    Serial.println("ERROR! NO DELTA Y FOUND. D: " + (String) straight_line_dist + " A: " + (String) orientation);
  }
  // if (0 <= orientation <= (PI * 0.5))
  // {
  //   return straight_line_dist * cos(orientation);
  // }
  // else if ((PI * 0.5) < orientation <= PI)
  // {
  //   return (straight_line_dist * cos(PI - orientation));
  // }
  // else if (-PI*0.5 > orientation > (-PI))
  // {
  //   return straight_line_dist * cos(orientation - PI);
  // }
  // else
  // {
  //   return straight_line_dist * cos((2 * PI) - orientation);
  // }
}

float EasyRobot::calc_delta_X(float straight_line_dist)
{
  if(orientation == 0){
    return 0; // No change in X
  } else if (orientation == (0.5*PI)) {
    return straight_line_dist; // No change in Y
  } else if (orientation == PI){
    return 0; // No chnage along X axis
  } else if (orientation == 1.5*PI){
    return -straight_line_dist; // No change in Y
  } else if (orientation > 0 && orientation < (0.5*PI)){
    return straight_line_dist * cosf(orientation);
  } else if (orientation > (0.5*PI) && orientation < PI){
    return straight_line_dist * cosf(orientation-(0.5*PI));
  } else if (orientation > PI && orientation < (1.5*PI)){
    return -(straight_line_dist * cosf(orientation-PI));
  } else if (orientation > (1.5*PI) && orientation < (2*PI)){
    return -(straight_line_dist * cosf(orientation-(1.5*PI)));
  } else{
    Serial.println("ERROR! NO DELTA X FOUND. D: " + (String) straight_line_dist + " A: " + (String) orientation);
  }
  // if (0 <= orientation <= (PI * 0.5))
  // {
  //   return straight_line_dist * sin(orientation);
  // }
  // else if ((PI * 0.5) < orientation <= PI)
  // {
  //   return (straight_line_dist * sin(PI - orientation));
  // }
  // else if (-PI*0.5 > orientation > (-PI))
  // {
  //   return straight_line_dist * sin(orientation - PI);
  // }
  // else
  // {
  //   return straight_line_dist * sin((2 * PI) - orientation);
  // }
}

// Move the robot forward
void EasyRobot::setupMoveForward(float distance)
{
  leftMotor.setTargetPositionInMillimeters(leftMotor.getCurrentPositionInMillimeters() + distance);
  rightMotor.setTargetPositionInMillimeters(rightMotor.getCurrentPositionInMillimeters() + distance);
}

// Turn the robot by a given angle (in radians)
void EasyRobot::setUpTurn(float TargetOrientation)
{
  float angle = TargetOrientation - getOrientation();
  float distance = angle * rotationConstant;
  leftMotor.setCurrentPositionInMillimeters(0L);
  rightMotor.setCurrentPositionInMillimeters(0L);
  leftMotor.setTargetPositionInMillimeters(distance);
  rightMotor.setTargetPositionInMillimeters(-distance);
  if (distance != 0)
  {
    ROTATE_MOVE = true;
  }
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

float EasyRobot::toDeg(float rad)
{
  return rad * (180.0 / PI);
}
float EasyRobot::toRad(float deg)
{
  return deg * (PI / 180.0);
}