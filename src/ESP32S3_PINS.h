// This Header is used to set the pin numbers


// Motor Pins
// S2
const int L_Stepper_STEP_PIN = 46;
const int L_Stepper_DIR_PIN = 13;
const int L_Stepper_ENABLE_PIN = 10;
const int L_Stepper_ENCODER_PIN = 47;
// S1
const int R_Stepper_STEP_PIN = 7;
const int R_Stepper_DIR_PIN = 6;
const int R_Stepper_ENABLE_PIN = 15;
const int R_Stepper_ENCODER_PIN = 21;

// Servo
const int SERVO_PIN = 16;

//  DC_Motor
const int L_DC_PWM_PIN = 0;
const int L_DC_IN1_PIN = 38;
const int L_DC_IN2_PIN = 37;

const int R_DC_PWM_PIN = 45;
const int R_DC_IN1_PIN = 36;
const int R_DC_IN2_PIN = 35;

const int DC_STBY_PIN = 48;


//  Line Sensors
const int IR1_PIN = 14;
const int IR2_PIN = 13;
const int IR3_PIN = 12;
const int IR4_PIN = 11;

//  Limit Switches
const int LIM1_PIN = 5;
const int LIM2_PIN = 4;

//  I2C Bus
const int TF_I2C_SCL_PIN = 17;
const int TF_I2C_SDA_PIN = 18;

//  SPI
const int MISO_PIN = 39;
const int MOSI_PIN = 40;
const int SPI_SCK_PIN = 41;
const int SD_CS_PIN = 42;

//  RESET
const int RST_PIN = 20;

void set_IO_pins_low(void){
    
pinMode(L_Stepper_STEP_PIN, OUTPUT);
pinMode(L_Stepper_DIR_PIN, OUTPUT);
pinMode(L_Stepper_ENABLE_PIN, OUTPUT);
pinMode(L_Stepper_ENCODER_PIN, INPUT);
pinMode(R_Stepper_STEP_PIN, OUTPUT);
pinMode(R_Stepper_DIR_PIN, OUTPUT);
pinMode(R_Stepper_ENABLE_PIN, OUTPUT);
pinMode(R_Stepper_ENCODER_PIN, INPUT);
pinMode(SERVO_PIN, OUTPUT);
pinMode(L_DC_PWM_PIN, OUTPUT);
pinMode(L_DC_IN1_PIN, OUTPUT);
pinMode(L_DC_IN2_PIN, OUTPUT);
pinMode(R_DC_PWM_PIN, OUTPUT);
pinMode(R_DC_IN1_PIN, OUTPUT);
pinMode(R_DC_IN2_PIN, OUTPUT);
pinMode(DC_STBY_PIN, OUTPUT);

digitalWrite(L_Stepper_STEP_PIN, LOW);
digitalWrite(L_Stepper_DIR_PIN, LOW);
digitalWrite(L_Stepper_ENABLE_PIN, LOW);
digitalWrite(R_Stepper_STEP_PIN, LOW);
digitalWrite(R_Stepper_DIR_PIN, LOW);
digitalWrite(R_Stepper_ENABLE_PIN, LOW);

digitalWrite(SERVO_PIN, LOW);
digitalWrite(L_DC_PWM_PIN, LOW);
digitalWrite(L_DC_IN1_PIN, LOW);
digitalWrite(L_DC_IN2_PIN, LOW);
digitalWrite(R_DC_PWM_PIN, LOW);
digitalWrite(R_DC_IN1_PIN, LOW);
digitalWrite(R_DC_IN2_PIN, LOW);
digitalWrite(DC_STBY_PIN, LOW);

pinMode(TF_I2C_SCL_PIN, OUTPUT);
pinMode(TF_I2C_SDA_PIN, OUTPUT);
pinMode(MISO_PIN, OUTPUT);
pinMode(MOSI_PIN, OUTPUT);
pinMode(SPI_SCK_PIN, OUTPUT);
pinMode(SD_CS_PIN, OUTPUT);
pinMode(RST_PIN, OUTPUT);

digitalWrite(TF_I2C_SCL_PIN, LOW);
digitalWrite(TF_I2C_SDA_PIN, LOW);
digitalWrite(MISO_PIN, LOW);
digitalWrite(MOSI_PIN, LOW);
digitalWrite(SPI_SCK_PIN, LOW);
digitalWrite(SD_CS_PIN, LOW);
digitalWrite(RST_PIN, HIGH);



pinMode(IR1_PIN, INPUT);
pinMode(IR2_PIN, INPUT);
pinMode(IR3_PIN, INPUT);
pinMode(IR4_PIN, INPUT);
pinMode(LIM1_PIN, INPUT);
pinMode(LIM2_PIN, INPUT);
digitalWrite(R_Stepper_ENCODER_PIN, INPUT);
digitalWrite(L_Stepper_ENCODER_PIN, INPUT);

}