#include <MPU6050_tockn.h>
#include <Wire.h>



MPU6050 mpu6050(Wire);





void MPU6050_SETUP() {

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  delay(100);
  Serial.println("MPU6050 calibrating...!!!!!");

}
