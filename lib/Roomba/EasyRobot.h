#include <Arduino.h>
#include <SpeedyStepperCustom.h>
#include <math.h>
#include <queue>

enum unit { KMH = 1, MMS = 2 };



class EasyRobot
{
  private:
    float xCoordinate;
    float yCoordinate;
    float orientation;
    bool L_MOVE_DONE;
    bool R_MOVE_DONE;
    bool ROTATE_MOVE;
    bool STRAIGHT_MOVE;

    float nextMoveOrientation;
    float nextMoveDiagDistance;
    std::queue<EasyRobot> move_queue;

    void updatePosition(float deltaX, float deltaY, float targetOrientation);
    void updatePosition(float deltaX, float deltaY);
    void updatePosition(float deltaOrientation);
    float calculateOrientation(float deltaX, float deltaY);

  public:
    
    SpeedyStepper leftMotor;
    SpeedyStepper rightMotor;
    EasyRobot();
    void begin(unit speed_units, float stepsPerMillimeters, float speed, float Acceleration);
    void setPosition(float angle);
    void setPosition(float xPos, float yPos);
    void setPosition(float xPos, float yPos, float angle);
    void setUpPins(int leftMotorStepPin, int leftMotorDirPin, int leftMotorEnablePin, int rightMotorStepPin, int rightMotorDirPin, int rightMotorEnablePin);
    void setUpTurn(float angle);
    void setUpTurnDeg(float angle);
    float getXCoordinate() const;
    float getYCoordinate() const;
    float getOrientation() const;
    void moveTo(float targetX, float targetY);
    void setupMoveForward(float distance);
    bool processMovement();
  
    void stop();

    void add_move(float targetX, float targetY);
    void add_rot_move(float deltaTheta);

    void setSpeedInKMH(float speed);
    void setSpeedInMMS(float speed);
    void setAccelerationInKMHH(float speed);
    void setAccelerationInMMSS(float speed);

};
