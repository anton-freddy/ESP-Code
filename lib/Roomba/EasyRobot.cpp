#include <EasyRobot.h>

const float rotationConstant = 160.49877; // Must be adjusted based on wheel distance and mounting points

void EasyRobot::updatePosition(float deltaX, float deltaY, float targetOrientation)
{
  xCoordinate += deltaX;
  yCoordinate += deltaY;
  orientation = targetOrientation;
}


// Constructor
EasyRobot::EasyRobot(int leftMotorStepPin, int leftMotorDirPin, int leftMotorEnablePin, int rightMotorStepPin, int rightMotorDirPin, int rightMotorEnablePin)
{
  leftMotor.connectToPins(leftMotorStepPin, leftMotorDirPin, leftMotorEnablePin);
  rightMotor.connectToPins(rightMotorStepPin, rightMotorDirPin, rightMotorEnablePin);
  xCoordinate = 0;
  yCoordinate = 0;
  orientation = 0;
  L_MOVE_DONE = false;
  R_MOVE_DONE = false;
}

// Initialize the robot, for unit pick either KMH or MMS
void EasyRobot::begin(unit speed_units, float stepsPerMillimeters, float speed, float Acceleration)
{

  leftMotor.setStepsPerMillimeter(stepsPerMillimeters);
  rightMotor.setStepsPerMillimeter(stepsPerMillimeters);

  if(speed_units == 1){
    setSpeedInKMH(speed);
    setAccelerationInKMHH(Acceleration);
  }else{
    setSpeedInMMS(speed);
    setAccelerationInMMSS(Acceleration);
  }

  leftMotor.setCurrentPositionInMillimeters(0L);
  rightMotor.setCurrentPositionInMillimeters(0L);

  leftMotor.enableStepper();
  rightMotor.enableStepper();
  
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

// Move the robot directly to a target position (X, Y)
void EasyRobot::moveTo(float targetX, float targetY)
    {
      float deltaX = targetX - xCoordinate;
      float deltaY = targetY - yCoordinate;
      float targetOrientation;


      if(deltaX == 0){
        if(deltaY > 0){
          targetOrientation = 0;
        }else{
          targetOrientation = PI;
        }
      }else if(deltaY == 0){
        if(deltaX > 0){
          targetOrientation = 0.5*PI;
        }else{
          targetOrientation = 1.5*PI;
        }
      }else{
        targetOrientation = atan(deltaX/deltaY);
      }
      float deltaOrientation = targetOrientation - orientation;



      // Rotate the robot to face the target

      turn(deltaOrientation);
      while(1){
        if(processMovement()){
          break;
        }
      }

      float distance_diagonal = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
      // Calculate the steps to move in the X and Y direction
      int stepsX = deltaX; // Adjust this conversion factor as needed
      int stepsY = deltaY; // Adjust this conversion factor as needed

      // Move the robot using absolute position
      leftMotor.setCurrentPositionInMillimeters(0L);
      rightMotor.setCurrentPositionInMillimeters(0L);
      leftMotor.setupMoveInMillimeters(distance_diagonal);
      rightMotor.setupMoveInMillimeters(distance_diagonal);

      // Update the position and orientation
      updatePosition(targetX, targetY, targetOrientation);
    }

bool EasyRobot::processMovement(){

  if(leftMotor.processMovement()){
    L_MOVE_DONE = true;
  }

  if(rightMotor.processMovement()){
    R_MOVE_DONE = true;
  }

  if(L_MOVE_DONE && R_MOVE_DONE){
    L_MOVE_DONE = false;
    R_MOVE_DONE = false;
    return true; 
  }else{
    return false;
  }
}

// Move the robot forward
void EasyRobot::moveForward(float distance)
{
  leftMotor.setupMoveInMillimeters(leftMotor.getCurrentPositionInMillimeters() + distance);
  rightMotor.setupMoveInMillimeters(rightMotor.getCurrentPositionInMillimeters() + distance);
}


// Turn the robot by a given angle (in radians)
void EasyRobot::turn(float angle)
{
  float newAngle;
  if(angle > 3.14159){
    newAngle = -3.14159 - ((3.14159-angle));
  }else{
    newAngle = angle;
  };

  float distance = newAngle * rotationConstant;
  leftMotor.setCurrentPositionInMillimeters(0L);
  rightMotor.setCurrentPositionInMillimeters(0L);
  leftMotor.setupMoveInMillimeters(distance);
  rightMotor.setupMoveInMillimeters(-distance);

}

void EasyRobot::turn_deg(float angle){
  turn(2*PI * (angle / 360));
}

// Stop the robot
void EasyRobot::stop()
{
  leftMotor.setCurrentPositionInSteps(0);
  rightMotor.setCurrentPositionInSteps(0);
}


// Sets the speed of the robot in KM/h, !!NOTE!! you must set the steps per millimeter before setting the speed
void EasyRobot::setSpeedInKMH(float speed){
    setSpeedInMMS(speed*277.7777777778);
}


// Sets the speed of the robot in mm/s, !!NOTE!! you must set the steps per millimeter before setting the speed
void EasyRobot::setSpeedInMMS(float speed){

    leftMotor.setSpeedInMillimetersPerSecond(speed);
    rightMotor.setSpeedInMillimetersPerSecond(speed);
}

// Sets the speed of the robot in KM/h, !!NOTE!! you must set the steps per millimeter before setting the speed
void EasyRobot::setAccelerationInKMHH(float speed){
    setAccelerationInMMSS(speed*0.2778);

}


// Sets the speed of the robot in mm/s, !!NOTE!! you must set the steps per millimeter before setting the speed
void EasyRobot::setAccelerationInMMSS(float speed){

    leftMotor.setAccelerationInMillimetersPerSecondPerSecond(speed);
    rightMotor.setAccelerationInMillimetersPerSecondPerSecond(speed);
}