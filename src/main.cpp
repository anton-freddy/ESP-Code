
#include <main.h>
#include <ERROR.h>

EasyRobot ROOMBA(WHEEL_CIRCUMFERENCE, WHEEL_DISTANCE, MICROSTEP, STEPPER_STEP_COUNT, GEAR_RATIO);

int loop1_previousMillis = 0;
int loop1_currentMillis = 0;

long previousMillis = 0;
long currentMillis = 0;

void setup()
{
  Serial.begin(115200);

  is_I2C_setup = setup_I2C();

  ROOMBA.setUpMotors(L_Stepper_STEP_PIN, L_Stepper_DIR_PIN, L_Stepper_ENABLE_PIN, R_Stepper_STEP_PIN, R_Stepper_DIR_PIN, R_Stepper_ENABLE_PIN, MS1_pin, MS2_pin, MS3_pin);
  ROOMBA.setUpEncoders();
  ROOMBA.begin(KMH, 10); // 19.1525439

  setupBumpers();
  // setup_LiDAR();

  // setup_IR();
  // setup_servo();

  Serial.println("END OF SETUP");
  // ROOMBA.enqueueMove(0, 3000);
  //  ROOMBA.enqueueMove(0, 0);

  // ROOMBA.update_stepper_DIR_pin();
  //  while (1);

  // ROOMBA.setUpMove(1000,1000);

  // ROOMBA.moveTo(1000,1000);

  delay(1000);
  // while(1);
}

bool clockwise = false;
//  Contorl Robot Movement
void loop()
{
  if (bump_triggred)
  {
    setBumpBackOFF();
    bump_triggred = false;
  }
  //tickServo();
  // move_servo();
  ROOMBA.loop();
  // ROOMBA.followHeading(PI, 5000);

  // loop1_currentMillis = millis();
  // if(loop1_currentMillis - loop1_previousMillis >= 50){
  //   loop1_previousMillis = loop1_currentMillis;
  //   ROOMBA.updatePose();
  // }

  // currentMillis = millis();
  // if (currentMillis > previousMillis + 100)
  // {
  //   previousMillis = currentMillis;
  //   String ArrayLine;
  //   ArrayLine = (String)millis() + "\t";
  //   ArrayLine += "X: " + (String)ROOMBA.getXCoordinate() + " Y: " + (String) ROOMBA.getYCoordinate() + " A: " + (String)ROOMBA.getOrientation();
  //   Serial.println(ArrayLine);
  // }
  // ROOMBA.UpdatePosFromEncoders(1);
  // currentMillis = millis();
  // if (currentMillis > previousMillis + 100)
  // {
  //   previousMillis = currentMillis;
  //   String ArrayLine;
  //   ArrayLine = (String)millis() + "\t";
  //   ArrayLine += "Lidar 1: " + (String)get_LiDAR_reading(LiDAR_1) + ", LiDAR 2: " + (String)get_LiDAR_reading(LiDAR_2) + ", Servo A: " + (String)SERVO_pos + "\t";
  //   ArrayLine += "L_ENC: " + (String)ROOMBA.L_getCurrentPos_CM() + ", R_ENC: " + (String)ROOMBA.R_getCurrentPos_CM();
  //   Serial.println(ArrayLine);
  // }

  // // if (!ROOMBA.motionComplete())
  // // {
  // //   ROOMBA.processMovement();
  // // }
}

void setup_LiDAR()
{
  if (!is_I2C_setup)
  {
    I2CA.begin(I2CA_SDA, I2CA_SCL);
  }

#ifdef ENABLE_LiDAR1

  while (bool flag = false)
  {
    if (LiDAR.Set_Frame_Rate(LiDAR_frame_rate, LiDAR_ADD_1))
    {

      flag = true;
    }
    else
    {
      send_ERROR(LiDAR_1, 0x00);
      break;
    }
  }

  while (bool flag = false)
  {
    if (LiDAR.Set_Trig_Mode(LiDAR_ADD_1))
    {

      flag = true;
    }
    else
    {
      send_ERROR(LiDAR_1, 0x01);
      break;
    }
  }
#endif

#ifdef ENABLE_LiDAR2

  while (bool flag = false)
  {
    if (LiDAR.Set_Frame_Rate(LiDAR_frame_rate, LiDAR_ADD_2))
    {
      flag = true;
    }
    else
    {
      send_ERROR(LiDAR_2, 0x00);
      break;
    }
  }
  while (bool flag = false)
  {
    if (LiDAR.Set_Trig_Mode(LiDAR_ADD_2))
    {

      flag = true;
    }
    else
    {
      send_ERROR(LiDAR_2, 0x01);
      break;
    }
  }
#endif

#ifdef ENABLE_LiDAR3

  while (bool flag = false)
  {
    if (LiDAR.Set_Frame_Rate(LiDAR_frame_rate, LiDAR_ADD_3))
    {
      flag = true;
    }
    else
    {
      send_ERROR(LiDAR_3, 0x00);
      break;
    }
  }
  while (bool flag = false)
  {
    if (LiDAR.Set_Trig_Mode(LiDAR_ADD_3))
    {

      flag = true;
    }
    else
    {
      send_ERROR(LiDAR_3, 0x01);
      break;
    }
  }
#endif

  Serial.println("LiDAR SETUP COMPLETE");
}

int16_t get_LiDAR_reading(int LiDAR_sel)
{
  int16_t LiDAR_ADDR;
  switch (LiDAR_sel)
  {
  case LiDAR_1:
    LiDAR_ADDR = LiDAR_ADD_1;
    break;

  case LiDAR_2:
    LiDAR_ADDR = LiDAR_ADD_2;
    break;

  case LiDAR_3:
    LiDAR_ADDR = LiDAR_ADD_3;
    break;
  default:
    break;
  }
  int16_t temp;

  // LiDAR.getData(temp, LiDAR_ADDR);
  // return temp;
  int i = 0;
  while (i < 10)
  {
    if (LiDAR.Set_Trigger(LiDAR_ADDR))
    {
      break;
    }
    i++;
  }
  if (i == 10)
  {
    send_ERROR(LiDAR_sel, 0x02);
    return 0;
  }
  else
  {
    i = 0;
    while (i < 10)
    {
      if (LiDAR.getData(temp, LiDAR_ADDR))
      {
        return temp;
      }
      i++;
    }
    if (i == 10)
    {
      send_ERROR(LiDAR_sel, 0x03);
      return 0;
    }
  }
}

void setup_IR()
{
  pinMode(L_IR_PIN, INPUT);
  pinMode(R_IR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(R_IR_PIN), IR1_ISR, RISING);
  // attachInterrupt(digitalPinToInterrupt(L_IR_PIN), IR2_ISR, RISING);
  Serial.println("IR SETUP COMPLETE");
}
bool get_IR1_status()
{
  int status = digitalRead(L_IR_PIN);
  if (status == HIGH)
  {
    return true;
  }
  else
  {
    return false;
  }
}
bool get_IR2_status()
{
  int status = digitalRead(R_IR_PIN);
  if (status == HIGH)
  {
    return true;
  }
  else if (status == LOW)
  {
    return false;
  }
}

void IR1_ISR()
{
  ROOMBA.stop();
}

void IR2_ISR()
{
  ROOMBA.stop();
}

void setup_servo()
{
  servo_pos = 90;
  writeIntToSlave(REG_SERVO_POS, servo_pos);
}

void writeToSlave(uint8_t regAddress, uint8_t data)
{
  I2CB.beginTransmission(slave_ADDR);
  I2CB.write(regAddress); // Register address
  I2CB.write(data);
  I2CB.endTransmission();
}

void writeIntToSlave(uint8_t regAddress, int data)
{
  uint8_t temp = data;
  I2CB.beginTransmission(slave_ADDR);
  I2CB.write(&regAddress, sizeof(uint8_t));
  I2CB.write(&temp, sizeof(uint8_t));
  I2CB.endTransmission();
}
void writeFloatToSlave(uint8_t regAddress, float data)
{
  int num, dec;
  float_to_ints(data, num, dec);
  uint8_t temp1 = num;
  uint8_t temp2 = dec;
  I2CB.beginTransmission(slave_ADDR);
  I2CB.write(regAddress);
  I2CB.write(&temp1, sizeof(uint8_t));
  I2CB.write(&temp2, sizeof(uint8_t));
  I2CB.endTransmission();
}

int requestIntFromSlave(uint8_t regAddress)
{
  I2CB.beginTransmission(slave_ADDR);
  I2CB.write(&regAddress, sizeof(uint8_t));
  I2CB.endTransmission();

  I2CB.requestFrom(slave_ADDR, sizeof(uint8_t));
  uint8_t read;
  I2CB.readBytes(&read, sizeof(uint8_t));
  return read;
}

float requestFloatFromSlave(uint8_t regAddress)
{
  I2CB.beginTransmission(slave_ADDR);
  I2CB.write(&regAddress, sizeof(uint8_t));
  I2CB.endTransmission();

  I2CB.requestFrom(slave_ADDR, 2 * sizeof(uint8_t));
  uint8_t num = 0;
  uint8_t dec = 0;
  I2CB.readBytes(&num, sizeof(uint8_t));
  I2CB.readBytes(&dec, sizeof(uint8_t));

  return ints_to_float(num, dec);
}

bool setup_I2C()
{
  if (I2CA.begin(I2CA_SDA, I2CA_SCL, 100000) && I2CB.begin(I2CB_SDA, I2CB_SCL, 100000))
  {
    return true;
  }
  else
    return false;
}

int getBatteryLevel()
{
  int analog = analogRead(BATTERY_LEVEL_PIN);
  float voltage = map_f(analog, 0, 4095, 0, 12.5);
  float voltage_12V = map_f(voltage, 0, 3.3, 0, 12);
  float percentage = map_f(voltage, 9, 12.5, 0, 100);
  return (int)percentage;
}

void bump_ISR()
{
  bump_triggred = true;
}

void setBumpBackOFF()
{
  Serial.println("BUMP TRIGRRED");
  ROOMBA.movementState = BACK_OFF;
  ROOMBA.obsticale_xpos = ROOMBA.getXCoordinate();
  ROOMBA.obsticale_ypos = ROOMBA.getYCoordinate();
  ROOMBA.clearMoves();
  ROOMBA.obsticale_avoiding = true;
  ROOMBA.obsticale_prev_orientation = ROOMBA.getOrientation();
  float arcLength = PI * obstciale_radius;
  int numWaypoints = int(arcLength / 50);
  float angleIncrement = PI / numWaypoints;
  float orientation = ROOMBA.getOrientation();
  for (int i = numWaypoints/2; i < numWaypoints; i++)
  {

    float deltaX = obstciale_radius * cos(orientation); // Calculate new x-coordinate increment
    float deltaY = obstciale_radius * sin(orientation); // Calculate new y-coordinate increment
    ROOMBA.enqueueMove(deltaX, deltaY);

        orientation += angleIncrement;
    // Handle orientation overflow
    if (orientation > 2 * PI)
    {
      orientation -= 2 * PI;
    }
    else if (orientation < 0)
    {
      orientation += 2 * PI;
    }

  }
  ROOMBA.backOff_previous_millis = millis();
  return;
}

void setupBumpers()
{
  pinMode(C_BUMP_PIN, INPUT);
  pinMode(R_BUMP_PIN, INPUT);
  pinMode(L_BUMP_PIN, INPUT);
  attachInterrupt(C_BUMP_PIN, bump_ISR, RISING);
  // attachInterrupt(R_BUMP_PIN, bump_ISR, HIGH);
  // attachInterrupt(L_BUMP_PIN, bump_ISR, HIGH);
}

void tickServo()
{
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis_Servo >= servo_delayBetweenSteps)
  {
    previousMillis_Servo = currentMillis;

    if (servo_pos <= 50 || servo_pos >= 130)
    {
      servo_increment = -servo_increment; // Reverse direction when reaching limits
    }

    servo_pos += servo_increment;
    writeIntToSlave(REG_SERVO_POS, servo_pos);
  }
}