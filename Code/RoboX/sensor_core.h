 ///////////////////////////////////////////////////////////////////////
/*----------------------------------------------------------------------
 * sensor_core.h
 *----------------------------------------------------------------------
 * 
 * Author:  Peter Nicholls
 * Created: 2015-05-31
 * Edited:  2015-05-31
 * 
 * The header file of the Sensor Core.
 * 
 *//////////////////////////////////////////////////////////////////////
 

#ifndef SENSOR_CORE_H
#define SENSOR_CORE_H


/// INCLUDES ///
#include "Arduino.h"
#include "config.h"
#include <inttypes.h>
#include <stdlib.h>
#include <Wire.h>

/// DEFINES ///
#define MPU9250_ADDRESS 0x68
#define MAG_ADDRESS 0x0C

#define MPU9250_ACC_CONFIG_REG 27
#define MPU9250_GYR_CONFIG_REG 28
#define MPU9250_ACC_X_DATA_REG 59 // All the following are 16 bits
#define MPU9250_ACC_Y_DATA_REG 61
#define MPU9250_ACC_Z_DATA_REG 63
#define MPU9250_TEMP_DATA_REG 65
#define MPU9250_GYR_X_DATA_REG 67
#define MPU9250_GYR_Y_DATA_REG 69
#define MPU9250_GYR_Z_DATA_REG 71
#define MPU9250_ACC_X_OFFSET_REG 119
#define MPU9250_ACC_Y_OFFSET_REG 122
#define MPU9250_ACC_Z_OFFSET_REG 125

#define MPU9250_FIFO_ENABLE_REG 35
#define MPU9250_FIFO_COUNT_REG 114
#define MPU9250_FIFO_READ_REG 116
#define MPU9250_INT_CONFIG_REG 55

#define GYRO_FULL_SCALE_250_DPS 0x00  
#define GYRO_FULL_SCALE_500_DPS 0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18

#define ACC_FULL_SCALE_2_G 0x00  
#define ACC_FULL_SCALE_4_G 0x08
#define ACC_FULL_SCALE_8_G 0x10
#define ACC_FULL_SCALE_16_G 0x18

#define IMU_BUFFER_SIZE 10
#define ACC_GYR_BUFFER_SIZE 14
#define MAG_BUFFER_SIZE 6


#define SHT_RANGE 0
#define MED_RANGE 1
#define LNG_RANGE 2

#define NOT_VALID -1
#define NOT_READ -2

/// STRUCTS ///
class InfraredSensor {
  private:
    uint8_t port;
    int8_t type;
    int16_t raw_value;
    int16_t value;
    void read_sht(void);
    void read_med(void);
    void read_lng(void);
  public:
    void initialize(uint8_t init_port, uint8_t init_type);
    void update(void);
    int16_t read(void);
};

class UltrasonicSensor {
  private:
    uint8_t trig;
    uint8_t echo;
    int32_t raw_value;
    int16_t value;
  public:
    void initialize(uint8_t init_trig, uint8_t init_echo);
    void update(void);
    int16_t read(void);
};

class DigitalSensor {
  private:
    uint8_t port;
    bool value;
  public:
  	void initialize(uint8_t init_port);
    void update(void);
    bool read(void);
};

class IMUSensor {
  private:
    int16_t values[IMU_BUFFER_SIZE];
  public:
  	void initialize(void);
    void update(void);
    bool read(void);
};


/// GLOBALS ///
extern InfraredSensor IR_SHT1; //to rename to appropriate locations
extern InfraredSensor IR_SHT2;
extern InfraredSensor IR_MED1;
extern InfraredSensor IR_MED2;
extern InfraredSensor IR_LNG1;
extern InfraredSensor IR_LNG2;
extern UltrasonicSensor USONIC1;
extern UltrasonicSensor USONIC2;
extern DigitalSensor IR_VAR1;
extern DigitalSensor IR_VAR2;
extern DigitalSensor IR_VAR3;
extern IMUSensor IMU;

/// FUNCTIONS ///
void init_sensor_core(void);
void update_sensors(void);



#endif
