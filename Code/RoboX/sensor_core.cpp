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
Sensor_t IR_SHT1 = {IR_SHT1_PIN, IR_SHT_RANGE, -1};
Sensor_t IR_SHT2 = {IR_SHT2_PIN, IR_SHT_RANGE, -1};
Sensor_t IR_MED1 = {IR_MED1_PIN, IR_MED_RANGE, -1};
Sensor_t IR_MED2 = {IR_MED2_PIN, IR_MED_RANGE, -1};
Sensor_t IR_LNG1 = {IR_LNG1_PIN, IR_LNG_RANGE, -1};
Sensor_t IR_LNG2 = {IR_LNG2_PIN, IR_LNG_RANGE, -1};
Sensor_t USONIC1 = {USONIC1_PIN, ULTRASONIC, -1};
Sensor_t USONIC2 = {USONIC2_PIN, ULTRASONIC, -1};

Sensor_t IR_VAR1 = {IR_VAR1_PIN, IR_VAR, -1};
Sensor_t IR_VAR2 = {IR_VAR2_PIN, IR_VAR, -1};
Sensor_t IR_VAR3 = {IR_VAR3_PIN, IR_VAR, -1};

int16_t IMU_BUFFER[IMU_BUFFER_SIZE] = {0};


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
  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS, MPU9250_ACC_CONFIG_REG, GYRO_FULL_SCALE_2000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS, MPU9250_GYR_CONFIG_REG, ACC_FULL_SCALE_16_G);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS, MPU9250_INT_CONFIG_REG ,0x02);
  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
  
  PRINTLN("done");
}


static void read_ir_sht(Sensor_t* to_read) {
  
  int16_t sensor_analog = analogRead(to_read->sensor_port);
  
  if (sensor_analog <= IR_SHT_MIN_ADC || sensor_analog >= IR_SHT_MAX_ADC) {
    sensor_analog = -1;
  }
  to_read->sensor_value = sensor_analog;
}

static void read_ir_med(Sensor_t* to_read) {
  
  int16_t sensor_analog = analogRead(to_read->sensor_port);
  
  if (sensor_analog <= IR_MED_MIN_ADC || sensor_analog >= IR_MED_MAX_ADC) {
    sensor_analog = -1;
  }
  to_read->sensor_value = sensor_analog;
}


static void read_ir_lng(Sensor_t* to_read) {
  
  int16_t sensor_analog = analogRead(to_read->sensor_port);
  
  if (sensor_analog <= IR_LNG_MIN_ADC || sensor_analog >= IR_LNG_MAX_ADC) {
    sensor_analog = -1;
  }
  to_read->sensor_value = sensor_analog;
}


static void read_ultrasonic(Sensor_t* to_read) {
  
  int16_t sensor_analog = analogRead(to_read->sensor_port);
  
  if (sensor_analog <= USONIC_MIN_ADC || sensor_analog >= USONIC_MAX_ADC) {
    sensor_analog = -1;
  }
  to_read->sensor_value = sensor_analog;
}


static void read_sonar(Sensor_t* to_read) {
  
  //int16_t sensor_analog = analogRead(to_read->sensor_port);
  
  int16_t sensor_analog = -1;
  to_read->sensor_value = sensor_analog;
}


static void sensor_distance(Sensor_t* to_read) {
  
  if (to_read->sensor_type == IR_SHT_RANGE) {
    read_ir_sht(to_read);
  }
  else if (to_read->sensor_type == IR_MED_RANGE) {
    read_ir_med(to_read);
  }
  else if (to_read->sensor_type == IR_LNG_RANGE) {
    read_ir_lng(to_read);
  }
  else if (to_read->sensor_type == ULTRASONIC) {
    read_ultrasonic(to_read);
  }
  else if (to_read->sensor_type == SONAR) {
    read_sonar(to_read);
  }
  
}


static void read_ir_var(Sensor_t* to_read) {
  
  bool sensor_actual = digitalRead(to_read->sensor_port);
  to_read->sensor_value = sensor_actual;
}


void sensor_detect(Sensor_t* to_read) {
  
  if (to_read->sensor_type == IR_VAR) {
    read_ir_var(to_read);
  }
  
}


void update_IMU(void) {
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
  sensor_distance(&IR_SHT1);
  sensor_distance(&IR_SHT2);
  sensor_distance(&IR_MED1);
  sensor_distance(&IR_MED2);
  sensor_distance(&IR_LNG1);
  sensor_distance(&IR_LNG2);
  sensor_distance(&USONIC1);
  sensor_distance(&USONIC2);
  
  sensor_detect(&IR_VAR1);
  sensor_detect(&IR_VAR2);
  sensor_detect(&IR_VAR3);
  
}


