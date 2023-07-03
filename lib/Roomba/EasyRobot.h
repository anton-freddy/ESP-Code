#include <Arduino.h>
#include <FlexyStepperCustom.h>
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

    // Process Movement Variables
    float previous_cord_LS;
    float previous_cord_RS;
    float current_cord_LS;
    float current_cord_RS;

    float nextMoveOrientation;
    float nextMoveDiagDistance;
    std::queue<EasyRobot> move_queue;

    void updatePosition(float deltaX, float deltaY, float targetOrientation);
    void updatePosition(float deltaX, float deltaY);
    void updatePosition(float deltaOrientation);
    float calculateOrientation(float deltaX, float deltaY);


  public:
    
    FlexyStepper leftMotor;
    FlexyStepper rightMotor;
    EasyRobot();
    void begin(unit speed_units, float stepsPerMillimeters, float speed, float Acceleration);
    void setPosition(float angle);
    void setPosition(float xPos, float yPos);
    void setPosition(float xPos, float yPos, float angle);
    void setUpPins(int leftMotorStepPin, int leftMotorDirPin, int leftMotorEnablePin, int rightMotorStepPin, int rightMotorDirPin, int rightMotorEnablePin);
    void setUpTurn(float TargetOrientation);
    void setUpTurnDeg(float TargetOrientation);
    float getXCoordinate() const;
    float getYCoordinate() const;
    float getOrientation() const;
    float calc_delta_Y(float straight_line_dist);
    float calc_delta_X(float straight_line_dist);
    void moveTo(float targetX, float targetY);
    void setupMoveForward(float distance);
    bool processMovement();
    bool motionComplete();

    float toDeg(float rad);
    float toRad(float deg);
  
    void stop();

    void add_move(float targetX, float targetY);
    void add_rot_move(float deltaTheta);

    void setSpeedInKMH(float speed);
    void setSpeedInMMS(float speed);
    void setAccelerationInKMHH(float speed);
    void setAccelerationInMMSS(float speed);

};
