/*
Reads an IMU and sends orientation & acceleration data to the serial.
*/




#include <Time.h>
#include "mpu.h"
#include "I2Cdev.h"




float refresh_rate_nominal = 100; // (Hz) How quicky should the loop iterate. 
unsigned long tStep;              // (us) What should the nominal time-step be?
unsigned long tCal  = 7;          // (us) Time-step correction.  
unsigned long tLastReading;       // (ms) Time at which the last IMU reading was taken.




void setup()
{
  delay(1000); // Allow time for the console to open.
  Serial.begin(500000);
  
  tStep = round(1000000.0 / refresh_rate_nominal);
  
  time_init();

  Fastwire::setup(400,0);
  mympu_open(refresh_rate_nominal);
}




void loop()
{
  unsigned long t0 = micros();
  
  tLastReading = millis();
  mympu_update();
  
  IMU_print();

  while((micros() - t0) < (tStep - tCal)) {}
}




void IMU_print()
{
  Serial.print((float) tLastReading * 0.001, 6);  Serial.print("\t"); 
  Serial.print(mympu.accel[0] * 9.81, 6);         Serial.print("\t"); 
  Serial.print(mympu.accel[1] * 9.81, 6);         Serial.print("\t");
  Serial.print(mympu.accel[2] * 9.81, 6);         Serial.print("\t");
  Serial.print(mympu.gyro[0] * PI/180, 6);        Serial.print("\t");
  Serial.print(mympu.gyro[1] * PI/180, 6);        Serial.print("\t");
  Serial.print(mympu.gyro[2] * PI/180, 6);        Serial.print("\t");
  Serial.print(mympu.quat[0], 6);                 Serial.print("\t");
  Serial.print(mympu.quat[1], 6);                 Serial.print("\t");
  Serial.print(mympu.quat[2], 6);                 Serial.print("\t");
  Serial.print(mympu.quat[3], 6);                 Serial.print("\n");
}
