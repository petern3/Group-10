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
UltrasonicSensor USONIC3;
SonarSensor SONAR1;

DigitalSensor IR_VAR1;
DigitalSensor IR_VAR2;
DigitalSensor IR_VAR3;
DigitalSensor LIMIT_O;
DigitalSensor TURN_ON;

DigitalSensor DIP8_S1;
DigitalSensor DIP8_S2;
DigitalSensor DIP8_S3;
DigitalSensor DIP8_S4;
DigitalSensor DIP8_S5;
DigitalSensor DIP8_S6;
DigitalSensor DIP8_S7;
DigitalSensor DIP8_S8;

IMUSensor IMU;
ColourSensor COLOUR;


//uint32_t USONIC_TIMEOUT_VALUE = USONIC_TIMEOUT;

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
  int8_t USONIC3_OFFSET_TEMP[2] = USONIC3_OFFSET;
  int8_t SONAR1_OFFSET_TEMP[2] = SONAR1_OFFSET;

  IR_SHT1.initialize(IR_SHT1_PIN, SHT_RANGE, IR_SHT1_OFFSET_TEMP, degrees_to_radians(IR_SHT1_ANGLE));
  IR_MED1.initialize(IR_MED1_PIN, MED_RANGE, IR_MED1_OFFSET_TEMP, degrees_to_radians(IR_MED1_ANGLE));
  IR_MED2.initialize(IR_MED2_PIN, MED_RANGE, IR_MED2_OFFSET_TEMP, degrees_to_radians(IR_MED2_ANGLE));
  IR_LNG1.initialize(IR_LNG1_PIN, LNG_RANGE, IR_LNG1_OFFSET_TEMP, degrees_to_radians(IR_LNG1_ANGLE));
  IR_LNG2.initialize(IR_LNG2_PIN, LNG_RANGE, IR_LNG2_OFFSET_TEMP, degrees_to_radians(IR_LNG2_ANGLE));
  USONIC1.initialize(USONIC1_TRIG_PIN, USONIC1_ECHO_PIN, USONIC1_OFFSET_TEMP, degrees_to_radians(USONIC1_ANGLE));
  USONIC2.initialize(USONIC2_TRIG_PIN, USONIC2_ECHO_PIN, USONIC2_OFFSET_TEMP, degrees_to_radians(USONIC2_ANGLE));
  USONIC3.initialize(USONIC3_TRIG_PIN, USONIC3_ECHO_PIN, USONIC3_OFFSET_TEMP, degrees_to_radians(USONIC3_ANGLE));
  SONAR1.initialize(SONAR1_PIN, SONAR1_OFFSET_TEMP, degrees_to_radians(SONAR1_ANGLE));
  
  IR_VAR1.initialize(IR_VAR1_PIN, LOW, INPUT);
  IR_VAR2.initialize(IR_VAR2_PIN, LOW, INPUT);
  IR_VAR3.initialize(IR_VAR3_PIN, LOW, INPUT);
  LIMIT_O.initialize(LIMIT_O_PIN, HIGH, INPUT_PULLUP);
  TURN_ON.initialize(TURN_ON_PIN, HIGH, INPUT_PULLUP);
  
  DIP8_S1.initialize(DIP8_S1_PIN, LOW, INPUT_PULLUP);
  DIP8_S2.initialize(DIP8_S2_PIN, LOW, INPUT_PULLUP);
  DIP8_S3.initialize(DIP8_S3_PIN, LOW, INPUT_PULLUP);
  DIP8_S4.initialize(DIP8_S4_PIN, LOW, INPUT_PULLUP);
  DIP8_S5.initialize(DIP8_S5_PIN, LOW, INPUT_PULLUP);
  DIP8_S6.initialize(DIP8_S6_PIN, LOW, INPUT_PULLUP);
  DIP8_S7.initialize(DIP8_S7_PIN, LOW, INPUT_PULLUP);
  DIP8_S8.initialize(DIP8_S8_PIN, LOW, INPUT_PULLUP);
  
  IMU.initialize();
  COLOUR.initialize();
  
  PRINTLN("done");
}

void update_sensors(void) {
  IR_SHT1.update();
  IR_MED1.update();
  IR_MED2.update();
  IR_LNG1.update();
  //IR_LNG2.update();
  
  USONIC1.update();
  USONIC2.update();
  USONIC3.update();
  
  //SONAR1.update();
}


void buffer_initialize(CircBuf_t* buffer, uint8_t size) {
  buffer->windex = 0;
  buffer->rindex = 0;
  buffer->size = size;
  buffer->data = (uint32_t *) calloc (size, sizeof(uint32_t));
  // Note use of calloc() to clear contents.
}

void buffer_store(CircBuf_t* buffer, uint32_t to_store){
  buffer->data[buffer->windex] = to_store;
  buffer->windex++;
  if (buffer->windex >= buffer->size) {
    buffer->windex = 0;
  }
}

uint32_t buffer_average(CircBuf_t buffer) {
  uint32_t sum = 0;
  for (uint8_t i=0; i < buffer.size; i++) {
    sum += buffer.data[i];
  }
  return sum / buffer.size;
}

uint32_t buffer_average_non_zero(CircBuf_t buffer) {
  uint32_t sum = 0;
  bool is_zero = false;
  for (uint8_t i=0; i < buffer.size; i++) {
    if (buffer.data[i]) {
      is_zero = true;
    }
    sum += buffer.data[i];
  }
  if (is_zero) {
    return 0;
  } else {
    return sum / buffer.size;
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
  buffer_initialize(&this->raw_value, SENSOR_BUFFER_SIZE);
}

void InfraredSensor::update(void) {
  /*int32_t sum = 0;
  for (uint8_t i=0; i < SENSOR_BUFFER_SIZE; i++) {
    sum += analogRead(this->pin);
  }
  this->raw_value = sum / SENSOR_BUFFER_SIZE;*/
  buffer_store(&this->raw_value, analogRead(this->pin));
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
      this->cart_value.y = NOT_VALID;
    } 
    else {
      this->cart_value = this->polar_value + this->offset;
    }
  }
  return this->cart_value;
}

void InfraredSensor::read_sht(void) {
  uint32_t average = buffer_average(this->raw_value);
  this->polar_value.r = average;

  PRINT(average);PRINT("   ");
  PRINT('\r');
  
  if (average <= IR_SHT_MIN_ADC && average > IR_SHT_DV1_ADC) {
    this->polar_value.r = map(this->polar_value.r, IR_SHT_MIN_ADC, IR_SHT_DV1_ADC, IR_SHT_MIN_MM, IR_SHT_DV1_MM);
  }
  else if (average <= IR_SHT_DV1_ADC && average > IR_SHT_DV2_ADC) {
    this->polar_value.r = map(this->polar_value.r, IR_SHT_DV1_ADC, IR_SHT_DV2_ADC, IR_SHT_DV1_MM, IR_SHT_DV2_MM);
  }
  else if (average <= IR_SHT_DV2_ADC && average >= IR_SHT_MAX_ADC) {
    this->polar_value.r = map(this->polar_value.r, IR_SHT_DV2_ADC, IR_SHT_MAX_ADC, IR_SHT_DV2_MM, IR_SHT_MAX_MM);
  } 
  else {
    this->polar_value.r = NOT_VALID;
  }
}

void InfraredSensor::read_med(void) {
  // ADC        Distance
  //  - \     /   800
  //     \   /    400
  //      \ /     200
  //  +    *      100
  
  uint32_t average = buffer_average(this->raw_value);
  this->polar_value.r = average;

  // Minimum range, largest ADC
  if (average <= IR_MED_MIN_ADC && average > IR_MED_DV1_ADC) {
    this->polar_value.r = map(this->polar_value.r, IR_MED_MIN_ADC, IR_MED_DV1_ADC, IR_MED_MIN_MM, IR_MED_DV1_MM);
  }
  else if (average <= IR_MED_DV1_ADC && average > IR_MED_DV2_ADC) {
    this->polar_value.r = map(this->polar_value.r, IR_MED_DV1_ADC, IR_MED_DV2_ADC, IR_MED_DV1_MM, IR_MED_DV2_MM);
  }
  else if (average <= IR_MED_DV2_ADC && average >= IR_MED_MAX_ADC) {
    this->polar_value.r = map(this->polar_value.r, IR_MED_DV2_ADC, IR_MED_MAX_ADC, IR_MED_DV2_MM, IR_MED_MAX_MM);
  } 
  else {
    this->polar_value.r = NOT_VALID;
  }
}

void InfraredSensor::read_lng(void) {
  uint32_t average = buffer_average(this->raw_value);
  if(average < 380){//adc value
  	average = NOT_VALID;
  }
  
  this->polar_value.r = average;
  
  //PRINT(average);PRINT("   ");
  //PRINT('\r');
  if (average <= IR_LNG_MIN_ADC && average > IR_LNG_DV1_ADC) {
    this->polar_value.r = map(this->polar_value.r, IR_LNG_MIN_ADC, IR_LNG_DV1_ADC, IR_LNG_MIN_MM, IR_LNG_DV1_MM);
  }
  else if (average <= IR_LNG_DV1_ADC && average > IR_LNG_DV2_ADC) {
    this->polar_value.r = map(this->polar_value.r, IR_LNG_DV1_ADC, IR_LNG_DV2_ADC, IR_LNG_DV1_MM, IR_LNG_DV2_MM);
  }
  else if (average <= IR_LNG_DV2_ADC && average >= IR_LNG_MAX_ADC) {
    this->polar_value.r = map(this->polar_value.r, IR_LNG_DV2_ADC, IR_LNG_MAX_ADC, IR_LNG_DV2_MM, IR_LNG_MAX_MM);
  } 
  else {
    this->polar_value.r = NOT_VALID;
  }
}

bool InfraredSensor::is_valid(void) {
  return polar_read().r != NOT_VALID;
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
  this->raw_value = pulseIn(this->echo_pin, HIGH, USONIC_TIMEOUT);
  
  this->polar_value.r = NOT_READ;
  this->cart_value.x = NOT_READ;
}

PolarVec UltrasonicSensor::polar_read(void) {
  if (this->polar_value.r == NOT_READ) {  // Only converts value once
    if (this->raw_value != 0) { // due to pulseIn function
      this->polar_value.r = this->raw_value / 5.8; // 29 / 2;  // microseconds to millimeters
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
      this->cart_value.y = NOT_VALID;
    } else {
      this->cart_value = this->polar_value + this->offset;
    }
  }
  return this->cart_value;
}

bool UltrasonicSensor::is_valid(void) {
  return polar_read().r != NOT_VALID;
}

////////////////////////////////////
/// SONAR SENSOR CLASS FUNCTIONS ///
////////////////////////////////////
void SonarSensor::initialize(uint8_t init_pin, int8_t init_offset[2], float init_angle) {
  this->pin = init_pin;
  this->offset = init_offset;
  this->polar_value.theta = init_angle;
  buffer_initialize(&this->raw_value, SENSOR_BUFFER_SIZE);
}

void SonarSensor::update(void) {
  
  buffer_store(&this->raw_value, analogRead(this->pin));
  this->polar_value.r = NOT_READ;
  this->cart_value.x = NOT_READ;
}

PolarVec SonarSensor::polar_read(void) {
  if (this->polar_value.r == NOT_READ) {  // Only converts value once
    uint32_t average = buffer_average(this->raw_value);
    this->polar_value.r = average;

    //PRINT(average); PRINT("  ");
    //PRINT('\r');
    // Minimum range, minimum ADC
    if (average >= SONAR_MIN_ADC && average < SONAR_DV1_ADC) {
      this->polar_value.r = map(this->polar_value.r, SONAR_MIN_ADC, SONAR_DV1_ADC, SONAR_MIN_MM, SONAR_DV1_MM);
    }
    else if (average >= SONAR_DV1_ADC && average < SONAR_DV2_ADC) {
      this->polar_value.r = map(this->polar_value.r, SONAR_DV1_ADC, SONAR_DV2_ADC, SONAR_DV1_MM, SONAR_DV2_MM);
    }
    else if (average >= SONAR_DV2_ADC && average <= SONAR_MAX_ADC) {
      this->polar_value.r = map(this->polar_value.r, SONAR_DV2_ADC, SONAR_MAX_ADC, SONAR_DV2_MM, SONAR_MAX_MM);
    }
    else {
      this->polar_value.r = NOT_VALID;
    }
    //PRINT(polar_value.r); PRINT("  ");
    //PRINT('\r');
  }
  return this->polar_value;
}

CartVec SonarSensor::cart_read(void) {
  if (this->cart_value.x == NOT_READ) {  // Only converts value once
    if (polar_read().r == NOT_VALID) {
      this->cart_value.x = NOT_VALID;
      this->cart_value.y = NOT_VALID;
    } else {
    this->cart_value = this->polar_value + this->offset;
    }
  }
  return this->cart_value;
}

bool SonarSensor::is_valid(void) {
  return polar_read().r != NOT_VALID;
}


///////////////////////////////////////
/// DIGIATAL SENSOR CLASS FUNCTIONS ///
///////////////////////////////////////
void DigitalSensor::initialize(uint8_t init_pin, bool init_active_state, uint8_t init_pinmode) {
  this->pin = init_pin;
  this->active_state = init_active_state;
  pinMode(init_pin, init_pinmode);
}

void DigitalSensor::update(void) {
  this->value = digitalRead(this->pin);
}

bool DigitalSensor::read(void) {
 return this->value;
}

bool DigitalSensor::is_active(void) {
  
  bool is_active_var = false;
  this->update();
  if (this->active_state == HIGH && this->value == true) {
    is_active_var = true;
  }
  else if (this->active_state == LOW && this->value == false){
    is_active_var = true;
  }
  return is_active_var;
}

///////////////////////////
/// IMU CLASS FUNCTIONS ///
///////////////////////////
void IMUSensor::initialize(void) {
  //this->values[IMU_BUFFER_SIZE] = {0};
  
  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS, MPU9250_ACC_CONFIG_REG, GYRO_FULL_SCALE_2000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS, MPU9250_GYR_CONFIG_REG, ACC_FULL_SCALE);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS, MPU9250_INT_CONFIG_REG ,0x02);
  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01); 
  
}

void IMUSensor::update(void) {
  
  uint8_t ACC_GYR_BUFFER[ACC_GYR_BUFFER_SIZE];  //The IMU reads in 8 bit data
  //uint8_t MAG_BUFFER[MAG_BUFFER_SIZE];
  
  I2Cread(MPU9250_ADDRESS, 0x3B, ACC_GYR_BUFFER_SIZE, ACC_GYR_BUFFER);
  
  // Accelerometer
  this->raw_values[0] = ACC_GYR_BUFFER[0]<<8 | ACC_GYR_BUFFER[1]; // Create 16 bits values from 8 bits data
  this->raw_values[1] = ACC_GYR_BUFFER[2]<<8 | ACC_GYR_BUFFER[3];
  this->raw_values[2] = ACC_GYR_BUFFER[4]<<8 | ACC_GYR_BUFFER[5];

  // Temperature
  //IMU_BUFFER[3] = ACC_GYR_BUFFER[7]<<8 | ACC_GYR_BUFFER[6];
  
  // Gyroscope
  this->raw_values[4] = ACC_GYR_BUFFER[8]<<8 | ACC_GYR_BUFFER[9];
  this->raw_values[5] = ACC_GYR_BUFFER[10]<<8 | ACC_GYR_BUFFER[11];
  this->raw_values[6] = ACC_GYR_BUFFER[12]<<8 | ACC_GYR_BUFFER[13];
  
  // Magnetometer
  /*uint8_t ST1;
  while (!(ST1&0x01)) {
    I2Cread(MAG_ADDRESS,0x02,1,&ST1);
  }
  PRINTLN("6");
  // Read magnetometer data
  I2Cread(MAG_ADDRESS, 0x03, MAG_BUFFER_SIZE, MAG_BUFFER);
  PRINTLN("7");
  // Request next magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
  
  this->raw_values[7] = MAG_BUFFER[1]<<8 | MAG_BUFFER[0];
  this->raw_values[8] = MAG_BUFFER[3]<<8 | MAG_BUFFER[2];
  this->raw_values[9] = MAG_BUFFER[5]<<8 | MAG_BUFFER[4];
  PRINTLN("8");*/

  this->values[0] = NOT_READ;
}

int16_t* IMUSensor::read(void) {
  if (this->values[0] == NOT_READ) {
    for (uint8_t i=0; i < IMU_BUFFER_SIZE; i++) {
      this->values[i] = this->raw_values[i];
    }
  }
  return this->values;
}

/////////////////////////////////////
/// COLOUR SENSOR CLASS FUNCTIONS ///
/////////////////////////////////////
void ColourSensor::initialize(void) {
  
  this->tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X);
  
  if (!tcs.begin()) {
    PRINTLN("colour sensor failed...");
    COLOUR_SENSOR_ERROR.activate();
    return;
  }
  
  tcs.setInterrupt(false);      // turn on LED
}

void ColourSensor::update(void) {
  if (!COLOUR_SENSOR_ERROR.is_active) {
    tcs.getRawData(&this->raw_values[0], &this->raw_values[1], &this->raw_values[2], &this->raw_values[3]);
    tcs.setInterrupt(true);  // turn off LED
    tcs.setInterrupt(false);      // turn on LED
    this->values[0] = NOT_READ;
  }
}

uint16_t* ColourSensor::read(void) {
  if (!COLOUR_SENSOR_ERROR.is_active) {
    if (this->values[0] == NOT_READ) {
      this->values[0] = this->raw_values[0];
      for (uint8_t i=1; i < COLOUR_BUFFER_SIZE; i++) {
        this->values[i] = uint16_t((float(this->raw_values[i]) / this->raw_values[0])*256);
      }
    }
    /*Serial.print("C:\t"); Serial.print(this->values[3]);
    Serial.print("\tR:\t"); Serial.print(this->values[0]);
    Serial.print("\tG:\t"); Serial.print(this->values[1]);
    Serial.print("\tB:\t"); Serial.print(this->values[2]);
    PRINTLN();*/
  }
  return this->values;
}




