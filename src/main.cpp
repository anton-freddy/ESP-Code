
#include <ESP32S3_PINS.h>
#include <ERROR.h>

TFLI2C LiDAR;
void setup_LiDAR();
int16_t get_LiDAR_reading(int LiDAR_sel);

void setup_IR();
bool get_IR1_status();
bool get_IR2_status();
void IR1_ISR();
void IR2_ISR();

Servo servo;
bool SERVO_CLOCKWISE = true;
int SERVO_pos = 0;
long servo_previousMillis = 0;
#define SERVO_INTERVAL 330
void setup_servo();
void move_servo();

EasyRobot ROOMBA(WHEEL_CIRCUMFERENCE, WHEEL_DISTANCE, MICROSTEP, STEPPER_STEP_COUNT, GEAR_RATIO);

int loop1_previousMillis = 0;
int loop1_currentMillis = 0;

long previousMillis = 0;
long currentMillis = 0;

void setup()
{
  Serial.begin(115200);
  ROOMBA.setUpMotors(L_Stepper_STEP_PIN, L_Stepper_DIR_PIN, L_Stepper_ENABLE_PIN, R_Stepper_STEP_PIN, R_Stepper_DIR_PIN, R_Stepper_ENABLE_PIN);
  ROOMBA.setUpEncoders(L_ENC_SDA, L_ENC_SCL, R_ENC_SDA, R_ENC_SCL);
  ROOMBA.begin(KMH, 3, 1000); // 19.1525439
  //setup_LiDAR();
  setup_IR();
  //setup_servo();

  Serial.println("END OF SETUP");
  ROOMBA.enqueueMove(0,2000);
  ROOMBA.enqueueMove(2000,3000);

  // ROOMBA.update_stepper_DIR_pin();
  //  while (1);
 
  //ROOMBA.setUpMove(1000,1000);
  
  // ROOMBA.moveTo(1000,1000);

  delay(1000);
  // while(1);
}

bool clockwise = false;
//  Contorl Robot Movement
void loop()
{
  //ROOMBA.resume();
  //ROOMBA.moveSteppers();
  // if(get_LiDAR_reading(LiDAR_1) < 15){
    
  //   if(!clockwise){
  //     ROOMBA.setUpTurn(PI);
  //     clockwise = true;
  //   } else {
  //     ROOMBA.setUpTurn(0);
  //     clockwise = false;
  //   }
    
  ROOMBA.loop();
  //ROOMBA.followHeading(PI, 5000);


  
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
  // LiDAR_I2C.begin(LiDAR_SDA_PIN, LiDAR_SCL_PIN, 100000);
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


  LiDAR.getData(temp, LiDAR_ADDR);
  return temp;
  int i = 0;
  while (i != 10 || !LiDAR.Set_Trigger(LiDAR_ADDR))
  {
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
    while (!LiDAR.getData(temp, LiDAR_ADDR) || i != 10)
    {
      i++;
    }
    if (i == 10)
    {
      send_ERROR(LiDAR_sel, 0x03);
    }
  }
  return temp;
}

// int16_t get_LiDAR2_reading()
// {
//   int16_t temp;
//   if (LiDAR.getData(temp, LiDAR_ADD_2))
//   {
//     return temp;
//   }
//   else
//   {
//     return 7777;
//   }
// }

void setup_IR()
{
  pinMode(IR1_PIN, INPUT);
  pinMode(IR2_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IR1_PIN), IR1_ISR, RISING);
  // attachInterrupt(digitalPinToInterrupt(IR2_PIN), IR2_ISR, RISING);
  Serial.println("IR SETUP COMPLETE");
}
bool get_IR1_status()
{
  int status = digitalRead(IR1_PIN);
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
  int status = digitalRead(IR2_PIN);
  if (status == HIGH)
  {
    return true;
  }
  else if ( status == LOW)
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
  SERVO_pos = 90;
  servo.attach(SERVO_PIN);
  SERVO_CLOCKWISE = true;
  servo.write(SERVO_pos);
  Serial.println("SERVO SETUP COMPLETE");
}
void move_servo()
{
  long servo_currentMillis = millis();
  if (servo_currentMillis - servo_previousMillis >= SERVO_INTERVAL)
  {
    servo_previousMillis = servo_currentMillis;
    if (SERVO_pos <= 45)
    {
      SERVO_CLOCKWISE = true;
    }
    else if (SERVO_pos >= 135)
    {
      SERVO_CLOCKWISE = false;
    }
    if (SERVO_CLOCKWISE)
    {
      SERVO_pos += 10;
    }
    else
    {
      SERVO_pos -= 10;
    }

    servo.write(SERVO_pos);
  }
}