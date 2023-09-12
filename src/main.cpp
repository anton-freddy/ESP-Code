
#include <ESP32S3_PINS.h>
#include <ERROR.h>
#include <SoftwareSerial.h>

enum RegisterAddress
{
  REG_SERVO_POS = 0x01,
  REG_CHARGE_LEVEL = 0x02,
  REG_XPOS = 0x03,
  REG_YPOS = 0x04,
  REG_APOS = 0x05
};

TFLI2C LiDAR;
void setup_LiDAR();
bool setup_I2C();
int16_t get_LiDAR_reading(int LiDAR_sel);

void setup_IR();
bool get_IR1_status();
bool get_IR2_status();
void IR1_ISR();
void IR2_ISR();
int getBatteryLevel();
void bump_ISR();
void setBumpBackOFF();


bool bump_triggred = false;
Servo servo;
bool SERVO_CLOCKWISE = true;
int SERVO_pos = 0;
long servo_previousMillis = 0;
#define SERVO_INTERVAL 200
bool is_I2C_setup = false;
void setup_servo();
void move_servo();

int requestIntFromSlave(uint8_t regAddress);
float requestFloatFromSlave(uint8_t regAddress);

void writeIntToSlave(uint8_t regAddress, int data);
void writeFloatToSlave(uint8_t regAddress, float data);

EasyRobot ROOMBA(WHEEL_CIRCUMFERENCE, WHEEL_DISTANCE, MICROSTEP, STEPPER_STEP_COUNT, GEAR_RATIO);

int loop1_previousMillis = 0;
int loop1_currentMillis = 0;

long previousMillis = 0;
long currentMillis = 0;

void setup()
{
  Serial.begin(115200);

  is_I2C_setup = setup_I2C();

  while (1)
  {
    writeIntToSlave(REG_SERVO_POS, 60);
    delay(500);
    Serial.println("POS: " + (String)requestIntFromSlave(REG_SERVO_POS));
    delay(500);
    writeIntToSlave(REG_SERVO_POS, 120);
    delay(500);
    Serial.println("POS: " + (String)requestIntFromSlave(REG_SERVO_POS));
    delay(500);
  }
  



  
  ROOMBA.begin(KMH, 5, 1000); // 19.1525439
  ROOMBA.setUpMotors(L_Stepper_STEP_PIN, L_Stepper_DIR_PIN, L_Stepper_ENABLE_PIN, R_Stepper_STEP_PIN, R_Stepper_DIR_PIN, R_Stepper_ENABLE_PIN, MS1_pin, MS2_pin, MS3_pin);
  ROOMBA.setUpEncoders();
  pinMode(IR3_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IR3_PIN), bump_ISR, RISING);
  
  // setup_LiDAR();

  // while(1){
  //   Serial.println("LiDAR1: " + (String)get_LiDAR_reading(LiDAR_1));
  //   delay(1000);
  //   Serial.println("LiDAR2: " + (String)get_LiDAR_reading(LiDAR_2));
  //   delay(1000);
  // }
  // setup_IR();
  //setup_servo();

  Serial.println("END OF SETUP");
  ROOMBA.enqueueMove(0, 3000);
  //ROOMBA.enqueueMove(0, 0);

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
  if(bump_triggred){
    setBumpBackOFF();
    bump_triggred = false;
  }
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
  if(!is_I2C_setup){
    I2CA.begin(I2CA_SDA, I2CA_SCL);
  }
  
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
  // while (bool flag = false)
  // {
  //   if (LiDAR.Set_Frame_Rate(LiDAR_frame_rate, LiDAR_ADD_3))
  //   {
  //     flag = true;
  //   }
  //   else
  //   {
  //     send_ERROR(LiDAR_3, 0x00);
  //     break;
  //   }
  // }
  // while (bool flag = false)
  // {
  //   if (LiDAR.Set_Trig_Mode(LiDAR_ADD_3))
  //   {

  //     flag = true;
  //   }
  //   else
  //   {
  //     send_ERROR(LiDAR_3, 0x01);
  //     break;
  //   }
  // }

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
  int pos = 45;
  bool INC = true;

  while (1)
  {
    if (pos <= 45)
    {
      INC = true;
    }
    if (pos >= 135)
    {
      INC = false;
    }
    if (INC)
    {
      pos++;
    }
    else
    {
      pos--;
    }
  
    // Serial.println("POS: " + (String)requestFromSlave(REG_CURRENT_POS));
    delay(5);
  }

  // while(1){
  //   byte error, address;
  // int nDevices;

  // Serial.println("Scanning...");

  // nDevices = 0;
  // for(address = 1; address < 127; address++ )
  // {
  //   // The i2c_scanner uses the return value of
  //   // the Write.endTransmisstion to see if
  //   // a device did acknowledge to the address.
  //   Wire.beginTransmission(address);
  //   error = Wire.endTransmission();

  //   if (error == 0)
  //   {
  //     Serial.print("I2C device found at address 0x");
  //     if (address<16)
  //       Serial.print("0");
  //     Serial.print(address,HEX);
  //     Serial.println("  !");

  //     nDevices++;
  //   }
  //   else if (error==4)
  //   {
  //     Serial.print("Unknown error at address 0x");
  //     if (address<16)
  //       Serial.print("0");
  //     Serial.println(address,HEX);
  //   }
  // }
  // if (nDevices == 0)
  //   Serial.println("No I2C devices found\n");
  // else
  //   Serial.println("done\n");

  // delay(5000);           // wait 5 seconds for next scan
  // }
}

void writeToSlave(uint8_t regAddress, uint8_t data)
{
  I2CB.beginTransmission(slave_ADDR);
  I2CB.write(regAddress); // Register address
  I2CB.write(data);
  I2CB.endTransmission();
}

void writeIntToSlave(uint8_t regAddress, int data){
  uint8_t temp = data;
  I2CB.beginTransmission(slave_ADDR);
  I2CB.write(&regAddress, sizeof(uint8_t));
  I2CB.write(&temp, sizeof(uint8_t));
  I2CB.endTransmission();
}
void writeFloatToSlave(uint8_t regAddress, float data){
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
  uint8_t read = 0;
  I2CB.readBytes(&read, sizeof(uint8_t));
  return read;

}

float requestFloatFromSlave(uint8_t regAddress)
{
  I2CB.beginTransmission(slave_ADDR);
  I2CB.write(&regAddress, sizeof(uint8_t));
  I2CB.endTransmission();

  I2CB.requestFrom(slave_ADDR, 2* sizeof(uint8_t));
  uint8_t num = 0;
  uint8_t dec = 0;
  I2CB.readBytes(&num, sizeof(uint8_t));
  I2CB.readBytes(&dec, sizeof(uint8_t));

  return ints_to_float(num, dec);
}


bool setup_I2C(){
  if(I2CA.begin(I2CA_SDA,I2CA_SCL, 100000) && I2CB.begin(I2CB_SDA,I2CB_SCL, 100000)){
    return true;

  }
  else return false;
}

int getBatteryLevel(){
  int analog = analogRead(IR4_PIN);
  float voltage = map_f(analogRead(IR4_PIN),0,1023,0,3.3);
  float volatge_12V = map_f(voltage,0,3.3,0,12);
  float percentage = map_f(volatge_12V,9,12,0,100);

  return (int)percentage;
}

void bump_ISR(){
  bump_triggred = true;
}

void setBumpBackOFF(){
  Serial.println("BUMP TRIGRRED");
  ROOMBA.movementState = BACK_OFF;
  ROOMBA.backOff_previous_millis = millis();
  return;
}