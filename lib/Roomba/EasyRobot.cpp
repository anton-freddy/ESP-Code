#include <EasyRobot.h>

const float rotationConstant = 171.931; //158.3;//160.49877; // Must be adjusted based on wheel distance and mounting points

void EasyRobot::updatePosition(float deltaX, float deltaY, float deltaOrientation)
{
  xCoordinate += deltaX;
  yCoordinate += deltaY;
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

  xCoordinate = 0;
  yCoordinate = 0;
  orientation = 0;
  L_MOVE_DONE = false;
  R_MOVE_DONE = false;
  ROTATE_MOVE = false;
  STRAIGHT_MOVE = false;
  
}

void EasyRobot::setUpPins(int leftMotorStepPin, int leftMotorDirPin, int leftMotorEnablePin, int rightMotorStepPin, int rightMotorDirPin, int rightMotorEnablePin){
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

  if((deltaX > 0) && (deltaY > 0)){         // Positive X, Positive Y - Quadrant 1
    return(atan(deltaX/deltaY));

  }else if((deltaX > 0) && (deltaY < 0)){   // Positive X, Negative Y - Quadrant 2
    return((PI/2) + ((PI/2) - atan(deltaX/deltaY)));

  }else if((deltaX < 0) && (deltaY < 0)){   // Negative X, Negative Y - Quadrant 3
    return(PI + atan(deltaX/deltaY));

  }else if((deltaX > 0) && (deltaY < 0)){   // Negative X, Positive Y - Quadrant 4
    return((2*PI) + atan(deltaX/deltaY));

  }else if(deltaX == 0){
    if(deltaY > 0){
      return(0);
    }else{
      return(PI);
    }
  }else{
    if(deltaX > 0){
      return(0.5*PI);
    }else{
      return(1.5*PI);
    }
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
      nextMoveDiagDistance = distance_diagonal;

      // Update the position and orientation
      updatePosition(deltaX, deltaY, deltaOrientation);
    }

bool EasyRobot::processMovement(){

  if(ROTATE_MOVE){
    float L_dist = leftMotor.getCurrentPositionInMillimeters();
    float R_dist = rightMotor.getCurrentPositionInMillimeters();
    float distance;
    if((L_dist * -1) == R_dist){
      distance = L_dist;
    }

    float FinalOrientation = distance/rotationConstant;
    float deltaOrientation = FinalOrientation - orientation;
    //updatePosition(0,0, deltaOrientation);
    if(orientation == nextMoveOrientation){
      ROTATE_MOVE = false;
    }

  }else{
    float L_dist = leftMotor.getCurrentPositionInMillimeters();
    float R_dist = rightMotor.getCurrentPositionInMillimeters();
    float deltaDistance;
    if((L_dist *-1) == R_dist){
      deltaDistance = L_dist;
    }
    
    float deltaX;
    float deltaY;

    if(0 <= orientation <= (PI*0.5) ){
      deltaX = deltaDistance*sin(orientation);
      deltaY = deltaDistance*cos(orientation);

    }else if((PI*0.5) < orientation <= PI){
      deltaX = deltaDistance*sin(PI-orientation);
      deltaY = deltaDistance*cos(PI-orientation);

    }else if(PI < orientation <= (PI*1.5)){
      deltaX = deltaDistance*sin(orientation-PI);
      deltaY = deltaDistance*cos(orientation-PI);
      
    }else{
      deltaX = deltaDistance*sin((2*PI)-orientation);
      deltaY = deltaDistance*cos((2*PI)-orientation);

    }

    //updatePosition(deltaX, deltaY, 0);
  }

  if(leftMotor.processMovement()){
    L_MOVE_DONE = true;
    leftMotor.setCurrentPositionInMillimeters(0L);
  }

  if(rightMotor.processMovement()){
    R_MOVE_DONE = true;
    rightMotor.setCurrentPositionInMillimeters(0L);
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
  if(angle > PI){
    newAngle = -PI + (PI - angle);
  }else if(angle < -PI){
    newAngle = PI - (-PI + angle);
  }else{
    newAngle = angle;
  }
  nextMoveOrientation = newAngle + orientation;
  float distance = newAngle * rotationConstant;
  leftMotor.setCurrentPositionInMillimeters(0L);
  rightMotor.setCurrentPositionInMillimeters(0L);
  leftMotor.setupMoveInMillimeters(distance);
  rightMotor.setupMoveInMillimeters(-distance);
  ROTATE_MOVE = true;

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