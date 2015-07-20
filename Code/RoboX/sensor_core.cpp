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
 

/// INCLUDES ///
#include "sensor_core.h"


/// GLOBALS ///
InfraredSensor IR_SHT1;
InfraredSensor IR_SHT2;
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


/// FUNCTIONS ///

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
  
  IR_SHT1.initialize(IR_SHT1_PIN, SHT_RANGE);
  IR_SHT2.initialize(IR_SHT2_PIN, SHT_RANGE);
  IR_MED1.initialize(IR_MED1_PIN, MED_RANGE);
  IR_MED2.initialize(IR_MED2_PIN, MED_RANGE);
  IR_LNG1.initialize(IR_LNG1_PIN, LNG_RANGE);
  IR_LNG2.initialize(IR_LNG2_PIN, LNG_RANGE);
  USONIC1.initialize(USONIC1_TRIG_PIN, USONIC1_ECHO_PIN);
  USONIC2.initialize(USONIC2_TRIG_PIN, USONIC2_ECHO_PIN);
  
  IR_VAR1.initialize(IR_VAR1_PIN);
  IR_VAR2.initialize(IR_VAR2_PIN);
  IR_VAR3.initialize(IR_VAR3_PIN);
  
  IMU.initialize();
  
  PRINTLN("done");
}


/// INFRARED SENSOR CLASS FUNCTIONS ///
void InfraredSensor::initialize(uint8_t init_port, uint8_t init_type) {
  this->port = init_port;
  this->type = init_type;
}

void InfraredSensor::update(void) {
  this->raw_value = analogRead(this->port);
  this->value = NOT_READ;
}

int16_t InfraredSensor::read(void) {
  if (this->value == NOT_READ) {  // Only converts value once
    if (type == SHT_RANGE) {
      this->read_sht();
    }
    else if (type == MED_RANGE) {
      this->read_med();
    }
    else if (type == LNG_RANGE) {
      this->read_lng();
    }
  }
 
 return this->value;
}


void InfraredSensor::read_sht(void) {
  
  if (this->raw_value >= IR_SHT_MIN_ADC || this->raw_value <= IR_SHT_MAX_ADC) {
    this->value = this->raw_value;
  } else {
    this->value = NOT_VALID;
  }
}

void InfraredSensor::read_med(void) {
  
  if (this->raw_value >= IR_MED_MIN_ADC || this->raw_value <= IR_MED_MAX_ADC) {
    this->value = this->raw_value;
  } else {
    this->value = NOT_VALID;
  }
}

void InfraredSensor::read_lng(void) {
  
  if (this->raw_value >= IR_LNG_MIN_ADC || this->raw_value <= IR_LNG_MAX_ADC) {
    this->value = this->raw_value;
  } else {
    this->value = NOT_VALID;
  }
}


/// ULTRASONIC SENSOR CLASS FUNCTIONS ///
void UltrasonicSensor::initialize(uint8_t init_trig, uint8_t init_echo) {
  this->trig = init_trig;
  this->echo = init_echo;
  pinMode(init_trig, OUTPUT);
  pinMode(init_echo, INPUT);
}

void UltrasonicSensor::update(void) {
  digitalWrite(this->trig, LOW);
  delayMicroseconds(2);
  digitalWrite(this->trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(this->trig, LOW);
  
  this->raw_value = pulseIn(this->echo, HIGH);
  this->value = NOT_READ;
}

int16_t UltrasonicSensor::read(void) {
  if (this->value == NOT_READ) {  // Only converts value once
    this->value = this->raw_value / 29 / 2;  // microseconds to centimeters
    if (this->value >= 1000) {
      this->value = -1;
    }
  }
 
 return this->value;
  
}


/// DIGIATAL SENSOR CLASS FUNCTIONS ///
void DigitalSensor::initialize(uint8_t init_port) {
  this->port = init_port;
}

void DigitalSensor::update(void) {
  this->value = digitalRead(this->port);
}

bool DigitalSensor::read(void) {
 return this->value;
  
}


/// IMU CLASS FUNCTIONS ///
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
  PRINTLN("1");
  uint8_t ACC_GYR_BUFFER[ACC_GYR_BUFFER_SIZE];  //The IMU reads in 8 bit data
  uint8_t MAG_BUFFER[MAG_BUFFER_SIZE];
  PRINTLN("2");
  //I2Cread(MPU9250_ADDRESS, 0x3B, ACC_GYR_BUFFER_SIZE, ACC_GYR_BUFFER);
  PRINTLN("3");
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


void update_sensors(void) {
  IR_SHT1.update();
  IR_SHT2.update();
  IR_MED1.update();
  IR_MED2.update();
  IR_LNG1.update();
  IR_LNG2.update();
  //USONIC1.update();
  //USONIC2.update();
  
  IR_VAR1.update();
  IR_VAR2.update();
  IR_VAR3.update();
  PRINTLN(USONIC1.read());
  IMU.update();
  
}


