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
  float delta_orientation = 0;

  if ((deltaX > 0) && (deltaY > 0))
  { // Positive X, Positive Y - Quadrant 1
    delta_orientation = (atan(deltaX / deltaY));
  }
  else if ((deltaX > 0) && (deltaY < 0))
  { // Positive X, Negative Y - Quadrant 2
    delta_orientation = ((PI / 2) + ((PI / 2) - atan(deltaX / deltaY)));
  }
  else if ((deltaX < 0) && (deltaY < 0))
  { // Negative X, Negative Y - Quadrant 3
    delta_orientation = (PI + atan(deltaX / deltaY));
  }
  else if ((deltaX > 0) && (deltaY < 0))
  { // Negative X, Positive Y - Quadrant 4
    delta_orientation = ((2 * PI) + atan(deltaX / deltaY));
  }
  else if (deltaX == 0)
  {
    if (deltaY > 0)
    {
      delta_orientation = (0 - getOrientation());
    }
    else
    {
      delta_orientation = (PI - getOrientation());
    }
  }
  else if (deltaY == 0)
  {
    if (deltaX > 0)
    {
      delta_orientation = ((0.5 * PI) - getOrientation());
    }
    else
    {
      delta_orientation = ((1.5 * PI) - getOrientation());
    }
  }

  if (delta_orientation <= -PI)
  {
    return (delta_orientation + PI) * -1;
  }
  else if (delta_orientation > PI)
  {
    return (delta_orientation - PI) * -1;
  }
  else
  {
    return delta_orientation;
  }
}
// Move the robot directly to a target position (X, Y)
void EasyRobot::moveTo(float targetX, float targetY)
{
  float deltaX = targetX - xCoordinate;
  float deltaY = targetY - yCoordinate;
  float targetOrientation = calculateOrientation(deltaX, deltaY);
  float deltaOrientation = targetOrientation - orientation;

  // if(deltaX == 0){
  //   if(deltaY > 0){
  //     targetOrientation = 0;
  //   }else{
  //     targetOrientation = PI;
  //   }
  // }else if(deltaY == 0){
  //   if(deltaX > 0){
  //     targetOrientation = 0.5*PI;
  //   }else{
  //     targetOrientation = 1.5*PI;
  //   }
  // }else{
  //   targetOrientation = atan(deltaX/deltaY);
  // }

  // Rotate the robot to face the target

  setUpTurn(deltaOrientation);
  while (1)
  {
    if (processMovement())
    {
      break;
    }
  }

  

  float distance_diagonal = sqrt(pow(deltaX, 2) + pow(deltaY, 2)); // Need to add signed to quadrant 3 & 4
  // Calculate the steps to move in the X and Y direction
  int stepsX = deltaX; // Adjust this conversion factor as needed
  int stepsY = deltaY; // Adjust this conversion factor as needed

  // Move the robot using absolute position
  leftMotor.setCurrentPositionInMillimeters(0L);
  rightMotor.setCurrentPositionInMillimeters(0L);
  leftMotor.setupMoveInMillimeters(distance_diagonal);
  rightMotor.setupMoveInMillimeters(distance_diagonal);
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

  if (ROTATE_MOVE)
  {
    float delta_orientatoion = delta_LS / rotationConstant;
    updatePosition(delta_orientatoion);
  }
  else
  {
    float delta_dist;
    if (delta_LS != delta_RS)
    {
      delta_dist = ((delta_LS + delta_RS) / 2.00);
    }
    else
    {
      delta_dist = delta_LS;
    }
    updatePosition(calc_delta_X(delta_dist, getOrientation()), calc_delta_Y(delta_dist, getOrientation()));
  }

  if (motionComplete())
  {
    leftMotor.setCurrentPositionInMillimeters(0L);
    rightMotor.setCurrentPositionInMillimeters(0L);
    leftMotor.setupMoveInMillimeters(0L);
    rightMotor.setupMoveInMillimeters(0L);
    ROTATE_MOVE = false;
    return true;
  }
  else
  {
    return false;
  }
}

float EasyRobot::calc_delta_Y(float straight_line_dist, float orientation)
{

  if (0 <= orientation <= (PI * 0.5))
  {
    return straight_line_dist * cos(orientation);
  }
  else if ((PI * 0.5) < orientation <= PI)
  {
    return (straight_line_dist * cos(PI - orientation));
  }
  else if (-PI*0.5 > orientation > (-PI))
  {
    return straight_line_dist * cos(orientation - PI);
  }
  else
  {
    return straight_line_dist * cos((2 * PI) - orientation);
  }
}

float EasyRobot::calc_delta_X(float straight_line_dist, float orientation)
{

  if (0 <= orientation <= (PI * 0.5))
  {
    return straight_line_dist * sin(orientation);
  }
  else if ((PI * 0.5) < orientation <= PI)
  {
    return (straight_line_dist * sin(PI - orientation));
  }
  else if (-PI*0.5 > orientation > (-PI))
  {
    return straight_line_dist * sin(orientation - PI);
  }
  else
  {
    return straight_line_dist * sin((2 * PI) - orientation);
  }
}

// Move the robot forward
void EasyRobot::setupMoveForward(float distance)
{
  leftMotor.setupMoveInMillimeters(leftMotor.getCurrentPositionInMillimeters() + distance);
  rightMotor.setupMoveInMillimeters(rightMotor.getCurrentPositionInMillimeters() + distance);
}

// Turn the robot by a given angle (in radians)
void EasyRobot::setUpTurn(float angle)
{
  float newAngle = angle;
  if (angle > PI)
  {
    newAngle = -PI + (PI - angle);
  }
  else if (angle < -PI)
  {
    newAngle = PI - (-PI + angle);
  }
  else
  {
    newAngle = angle;
  }
  nextMoveOrientation = newAngle + orientation;
  float distance = newAngle * rotationConstant;
  leftMotor.setCurrentPositionInMillimeters(0L);
  rightMotor.setCurrentPositionInMillimeters(0L);
  leftMotor.setupMoveInMillimeters(distance);
  rightMotor.setupMoveInMillimeters(-distance);
  if (newAngle != 0)
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
  leftMotor.setupStop();
  rightMotor.setupStop();
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