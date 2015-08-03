 ///////////////////////////////////////////////////////////////////////
/*----------------------------------------------------------------------
 * sensor_core.cpp
 *----------------------------------------------------------------------
 * 
 * Author:  Peter Nicholls
 * Created: 2015-05-31
 * Edited:  2015-05-31
 * 
 * Blurb
 * 
 *//////////////////////////////////////////////////////////////////////
 
////////////////
/// INCLUDES ///
////////////////
#include "sensor_core.h"
#include "exception_core.h"

///////////////
/// GLOBALS ///
///////////////
InfraredSensor IR_SHT1;
InfraredSensor IR_MED1;
InfraredSensor IR_MED2;
InfraredSensor IR_LNG1;
InfraredSensor IR_LNG2;
UltrasonicSensor USONIC1;
UltrasonicSensor USONIC2;

DigitalSensor IR_VAR1;
DigitalSensor IR_VAR2;
DigitalSensor IR_VAR3;

IMUSensor IMU;
ColourSensor COLOUR;

int8_t NO_WEIGHT[2] = {-1, -1};
CartVec weight_location = {-1, -1};

uint32_t USONIC_TIMEOUT_VALUE = USONIC_TIMEOUT;

/////////////////
/// FUNCTIONS ///
/////////////////
static void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data) {
  // This function read Nbytes bytes from I2C device at address Address. 
  // Put read bytes starting at register Register in the Data array. 

  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
 
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available()) {
    Data[index++]=Wire.read();
  }
}
 
static void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data) {
  // Write a byte (Data) in device (Address) at register (Register)
  
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

void init_sensor_core(void) {
  PRINT("\tSensors...");
  analogReference(INTERNAL2V56);
  Wire.begin();

  int8_t IR_SHT1_OFFSET_TEMP[2] = IR_SHT1_OFFSET;
  int8_t IR_MED1_OFFSET_TEMP[2] = IR_MED1_OFFSET;
  int8_t IR_MED2_OFFSET_TEMP[2] = IR_MED2_OFFSET;
  int8_t IR_LNG1_OFFSET_TEMP[2] = IR_LNG1_OFFSET;
  int8_t IR_LNG2_OFFSET_TEMP[2] = IR_LNG2_OFFSET;
  int8_t USONIC1_OFFSET_TEMP[2] = USONIC1_OFFSET;
  int8_t USONIC2_OFFSET_TEMP[2] = USONIC2_OFFSET;

  IR_SHT1.initialize(IR_SHT1_PIN, SHT_RANGE, IR_SHT1_OFFSET_TEMP, IR_SHT1_ANGLE);
  IR_MED1.initialize(IR_MED1_PIN, MED_RANGE, IR_MED1_OFFSET_TEMP, IR_MED1_ANGLE);
  IR_MED2.initialize(IR_MED2_PIN, MED_RANGE, IR_MED2_OFFSET_TEMP, IR_MED2_ANGLE);
  IR_LNG1.initialize(IR_LNG1_PIN, LNG_RANGE, IR_LNG1_OFFSET_TEMP, IR_LNG1_ANGLE);
  IR_LNG2.initialize(IR_LNG2_PIN, LNG_RANGE, IR_LNG2_OFFSET_TEMP, IR_LNG2_ANGLE);
  USONIC1.initialize(USONIC1_TRIG_PIN, USONIC1_ECHO_PIN, USONIC1_OFFSET_TEMP, USONIC1_ANGLE);
  USONIC2.initialize(USONIC2_TRIG_PIN, USONIC2_ECHO_PIN, USONIC1_OFFSET_TEMP, USONIC1_ANGLE);
  
  IR_VAR1.initialize(IR_VAR1_PIN);
  IR_VAR2.initialize(IR_VAR2_PIN);
  IR_VAR3.initialize(IR_VAR3_PIN);
  
  IMU.initialize();
  COLOUR.initialize();
  
  PRINTLN("done");
}

void update_sensors(void) {
  IR_SHT1.update();
  IR_MED1.update();
  IR_MED2.update();
  IR_LNG1.update();
  IR_LNG2.update();
  
  USONIC1.update();
  USONIC2.update();
  
  IR_VAR1.update();
  IR_VAR2.update();
  IR_VAR3.update();
  
  //IMU.update();
  //COLOUR.update();
}

bool weight_detect(void) {

  // Check left. No need for abs as the lower one (USONIC) should always be less.
  if (IR_MED1.polar_read().r - USONIC1.polar_read().r > WEIGHT_DETECT_TOLERANCE) {
    weight_location = USONIC1.cart_read();
    return true;
  }
  // Check right
  else if (IR_MED2.polar_read().r - USONIC2.polar_read().r > WEIGHT_DETECT_TOLERANCE) {
    weight_location = USONIC2.cart_read();
    return true;
  }
  else {
    weight_location = NO_WEIGHT;
    return false;
  }
}


///////////////////////////////////////
/// INFRARED SENSOR CLASS FUNCTIONS ///
///////////////////////////////////////
void InfraredSensor::initialize(uint8_t init_pin, uint8_t init_type, int8_t init_offset[2], float init_angle) {
  this->pin = init_pin;
  this->type = init_type;
  this->offset = init_offset;
  this->polar_value.theta = init_angle;
}

void InfraredSensor::update(void) {
  this->raw_value = analogRead(this->pin);
  this->polar_value.r = NOT_READ;
  this->cart_value.x = NOT_READ;
}

PolarVec InfraredSensor::polar_read(void) {
  if (this->polar_value.r == NOT_READ) {  // Only converts value once
    if (this->type == SHT_RANGE) {
      this->read_sht();
    }
    else if (this->type == MED_RANGE) {
      this->read_med();
    }
    else if (this->type == LNG_RANGE) {
      this->read_lng();
    }
  }
  return this->polar_value;
}

CartVec InfraredSensor::cart_read(void) {
  if (this->cart_value.x == NOT_READ) {  // Only converts value once
    if (polar_read().r == NOT_VALID) {
      this->cart_value.x = NOT_VALID;
    } else {
    this->cart_value = this->polar_value + this->offset;
    }
  }
  return this->cart_value;
}

void InfraredSensor::read_sht(void) {
  this->polar_value.r = this->raw_value;
  
  if (this->raw_value >= IR_SHT_MIN_ADC && this->raw_value < IR_SHT_DV1_ADC) {
    //map(this->polar_value.r, IR_SHT_MIN_ADC, IR_SHT_DV1_ADC, IR_SHT_MIN_MM, IR_SHT_DV1_MM);
  }
  else if (this->raw_value >= IR_SHT_DV1_ADC && this->raw_value < IR_SHT_DV2_ADC) {
    //map(this->polar_value.r, IR_SHT_DV1_ADC, IR_SHT_DV2_ADC, IR_SHT_DV1_MM, IR_SHT_DV2_MM);
  }
  else if (this->raw_value >= IR_SHT_DV2_ADC && this->raw_value < IR_SHT_MAX_ADC) {
    //map(this->polar_value.r, IR_SHT_DV2_ADC, IR_SHT_MAX_ADC, IR_SHT_DV2_MM, IR_SHT_MAX_MM);
  } else {
    this->polar_value.r = NOT_VALID;
  }
}

void InfraredSensor::read_med(void) {
  this->polar_value.r = this->raw_value;
  
  if (this->raw_value >= IR_MED_MIN_ADC && this->raw_value < IR_MED_DV1_ADC) {
    //map(this->polar_value.r, IR_MED_MIN_ADC, IR_MED_DV1_ADC, IR_MED_MIN_MM, IR_MED_DV1_MM);
  }
  else if (this->raw_value >= IR_MED_DV1_ADC && this->raw_value < IR_MED_DV2_ADC) {
    //map(this->polar_value.r, IR_MED_DV1_ADC, IR_MED_DV2_ADC, IR_MED_DV1_MM, IR_MED_DV2_MM);
  }
  else if (this->raw_value >= IR_MED_DV2_ADC && this->raw_value < IR_MED_MAX_ADC) {
    //map(this->polar_value.r, IR_MED_DV2_ADC, IR_MED_MAX_ADC, IR_MED_DV2_MM, IR_MED_MAX_MM);
  } else {
    this->polar_value.r = NOT_VALID;
  }
}

void InfraredSensor::read_lng(void) {
  this->polar_value.r = this->raw_value;
  
  if (this->raw_value >= IR_LNG_MIN_ADC && this->raw_value < IR_LNG_DV1_ADC) {
    //map(this->polar_value.r, IR_LNG_MIN_ADC, IR_LNG_DV1_ADC, IR_LNG_MIN_MM, IR_LNG_DV1_MM);
  }
  else if (this->raw_value >= IR_LNG_DV1_ADC && this->raw_value < IR_LNG_DV2_ADC) {
    //map(this->polar_value.r, IR_LNG_DV1_ADC, IR_LNG_DV2_ADC, IR_LNG_DV1_MM, IR_LNG_DV2_MM);
  }
  else if (this->raw_value >= IR_LNG_DV2_ADC && this->raw_value < IR_LNG_MAX_ADC) {
    //map(this->polar_value.r, IR_LNG_DV2_ADC, IR_LNG_MAX_ADC, IR_LNG_DV2_MM, IR_LNG_MAX_MM);
  } else {
    this->polar_value.r = NOT_VALID;
  }
}


/////////////////////////////////////////
/// ULTRASONIC SENSOR CLASS FUNCTIONS ///
/////////////////////////////////////////
void UltrasonicSensor::initialize(uint8_t init_trig_pin, uint8_t init_echo_pin, int8_t init_offset[2], float init_angle) {
  this->trig_pin = init_trig_pin;
  this->echo_pin = init_echo_pin;
  this->offset = init_offset;
  this->polar_value.theta = init_angle;
  
  pinMode(init_trig_pin, OUTPUT);
  pinMode(init_echo_pin, INPUT);
}

void UltrasonicSensor::update(void) {
  digitalWrite(this->trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(this->trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(this->trig_pin, LOW);
  
  this->raw_value = pulseIn(this->echo_pin, HIGH, USONIC_TIMEOUT_VALUE);
  this->polar_value.r = NOT_READ;
  this->cart_value.x = NOT_READ;
}

PolarVec UltrasonicSensor::polar_read(void) {
  if (this->polar_value.r == NOT_READ) {  // Only converts value once
    if (this->raw_value < USONIC_TIMEOUT) { // to check
      this->polar_value.r = this->raw_value / 29 / 2;  // microseconds to centimeters
    } else {
      this->polar_value.r = NOT_VALID;
    }
  }
  return this->polar_value;
}

CartVec UltrasonicSensor::cart_read(void) {
  if (this->cart_value.x == NOT_READ) {  // Only converts value once
    if (polar_read().r == NOT_VALID) {
      this->cart_value.x = NOT_VALID;
    } else {
    this->cart_value = this->polar_value + this->offset;
    }
  }
  return this->cart_value;
}

///////////////////////////////////////
/// DIGIATAL SENSOR CLASS FUNCTIONS ///
///////////////////////////////////////
void DigitalSensor::initialize(uint8_t init_pin) {
  this->pin = init_pin;
}

void DigitalSensor::update(void) {
  this->value = digitalRead(this->pin);
}

bool DigitalSensor::read(void) {
 return this->value;
  
}

///////////////////////////
/// IMU CLASS FUNCTIONS ///
///////////////////////////
void IMUSensor::initialize(void) {
  //this->values[IMU_BUFFER_SIZE] = {0};
  
  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS, MPU9250_ACC_CONFIG_REG, GYRO_FULL_SCALE_2000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS, MPU9250_GYR_CONFIG_REG, ACC_FULL_SCALE_16_G);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS, MPU9250_INT_CONFIG_REG ,0x02);
  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01); 
  
}

void IMUSensor::update(void) {
  //PRINTLN("1");
  uint8_t ACC_GYR_BUFFER[ACC_GYR_BUFFER_SIZE];  //The IMU reads in 8 bit data
  uint8_t MAG_BUFFER[MAG_BUFFER_SIZE];
  //PRINTLN("2");
  //I2Cread(MPU9250_ADDRESS, 0x3B, ACC_GYR_BUFFER_SIZE, ACC_GYR_BUFFER);
  //PRINTLN("3");
  // Accelerometer
  /*IMU_BUFFER[0] = ACC_GYR_BUFFER[0]<<8 | ACC_GYR_BUFFER[1]; // Create 16 bits values from 8 bits data
  IMU_BUFFER[1] = ACC_GYR_BUFFER[2]<<8 | ACC_GYR_BUFFER[3];
  IMU_BUFFER[2] = ACC_GYR_BUFFER[4]<<8 | ACC_GYR_BUFFER[5];
  PRINTLN("4");
  // Temperature
  //IMU_BUFFER[3] = ACC_GYR_BUFFER[7]<<8 | ACC_GYR_BUFFER[6];
  /*
  // Gyroscope
  IMU_BUFFER[4] = ACC_GYR_BUFFER[8]<<8 | ACC_GYR_BUFFER[9];
  IMU_BUFFER[5] = ACC_GYR_BUFFER[10]<<8 | ACC_GYR_BUFFER[11];
  IMU_BUFFER[6] = ACC_GYR_BUFFER[12]<<8 | ACC_GYR_BUFFER[13];
  PRINTLN("5");
  // Magnetometer
  uint8_t ST1;
  while (!(ST1&0x01)) {
    I2Cread(MAG_ADDRESS,0x02,1,&ST1);
  }
  
  // Read magnetometer data
  I2Cread(MAG_ADDRESS, 0x03, MAG_BUFFER_SIZE, MAG_BUFFER);
  
  // Request next magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
  
  IMU_BUFFER[7] = MAG_BUFFER[1]<<8 | MAG_BUFFER[0];
  IMU_BUFFER[8] = MAG_BUFFER[3]<<8 | MAG_BUFFER[2];
  IMU_BUFFER[9] = MAG_BUFFER[5]<<8 | MAG_BUFFER[4];*/
  
  //print_buffer(IMU_BUFFER, IMU_BUFFER_SIZE);
}

/////////////////////////////////////
/// COLOUR SENSOR CLASS FUNCTIONS ///
/////////////////////////////////////
void ColourSensor::initialize(void) {
  
  this->tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
  
  if (!tcs.begin()) {
    PRINTLN("colour sensor failed...");
    COLOUR_SENSOR_ERROR.activate();
    return;
  }
  
  tcs.setInterrupt(false);      // turn on LED
}

void ColourSensor::update(void) {
  if (!COLOUR_SENSOR_ERROR.is_active) {
    tcs.setInterrupt(false);      // turn on LED
    delay(60);
    tcs.getRawData(&this->raw_values[0], &this->raw_values[1], &this->raw_values[2], &this->raw_values[3]);
    tcs.setInterrupt(true);  // turn off LED
  }
}

uint16_t* ColourSensor::read(void) {
  if (!COLOUR_SENSOR_ERROR.is_active) {
    Serial.print("C:\t"); Serial.print(this->raw_values[3]);
    Serial.print("\tR:\t"); Serial.print(this->raw_values[1]);
    Serial.print("\tG:\t"); Serial.print(this->raw_values[2]);
    Serial.print("\tB:\t"); Serial.print(this->raw_values[3]);
    PRINTLN();
  }
  return this->raw_values;
}




