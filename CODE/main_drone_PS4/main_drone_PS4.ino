#include "PS4.h"
#include "MPU6050.h"



float roll_sample_angle = 0.0, pitch_sample_angle = 0.0, yaw_sample_angle = 0.0;
float roll_angle = 0.0, pitch_angle = 0.0, yaw_angle = 0.0;

float roll_sample_gyro = 0.0, pitch_sample_gyro = 0.0, yaw_sample_gyro = 0.0;
float roll_gyro = 0.0, pitch_gyro = 0.0, yaw_gyro = 0.0;


float esc_1, esc_2, esc_3, esc_4;
int throttle = 127;
int roll = 127;
int pitch = 127;
int yaw = 127;
bool drone = false;


float angle_roll_input = 0, angle_pitch_input = 0, angle_yaw_input = 0;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint = 0, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint = 0, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint = 0, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float pid_p_gain_roll = 0.15;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0;              //Gain setting for the roll I-controller
float pid_d_gain_roll =  0;              //Gain setting for the roll D-controller
int pid_max_roll = 50;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = 0.2;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 0.15;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 50;                     //Maximum output of the PID-controller (+/-)



void setup() {


  Serial.begin(115200);
  PS4_SETUP();
  MPU6050_SETUP();

  lights(1);

  for (int x = 0; x < 2000; x++) {
    mpu6050.update();
    roll_sample_angle +=  mpu6050.getAngleY();
    pitch_sample_angle +=  mpu6050.getAngleX();
    yaw_sample_angle += (mpu6050.getAngleZ() * -1);


    roll_sample_gyro +=  mpu6050.getGyroY();
    pitch_sample_gyro +=  mpu6050.getGyroX();
    yaw_sample_gyro +=  (mpu6050.getGyroZ() * -1);

  }

  roll_sample_angle /=  2000;
  pitch_sample_angle /= 2000;
  yaw_sample_angle /= 2000;

  roll_sample_gyro /=  2000;
  pitch_sample_gyro /=  2000;
  yaw_sample_gyro /=  2000;

  lights(0);

  Serial.println("MPU6050 calibrated");

  //    Serial.println();
  //  Serial.println();
  //  delay(1000);
  //
  //  Serial.print(roll_sample_angle);
  //  Serial.print("\t");
  //  Serial.print(pitch_sample_angle);
  //  Serial.print("\t");
  //  Serial.println(yaw_sample_angle);
  //  Serial.println();
  //  Serial.println();
  //  delay(1000);
  //
  //
  //  Serial.print(roll_sample_gyro);
  //  Serial.print("\t");
  //  Serial.print(pitch_sample_gyro);
  //  Serial.print("\t");
  //  Serial.println(yaw_sample_gyro);
  //  delay(5000);


}
void calculate_pid() {
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if (pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if (pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if (pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if (pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if (pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if (pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if (pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if (pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if (pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if (pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if (pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if (pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}

void loop() {

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //ANGLE & GYRO INPUTS
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  mpu6050.update();

  roll_angle = mpu6050.getAngleY() - roll_sample_angle;
  pitch_angle = mpu6050.getAngleX() - pitch_sample_angle;
  yaw_angle = (mpu6050.getAngleZ() * -1) - yaw_sample_angle;

  roll_gyro =  mpu6050.getGyroY() - roll_sample_gyro;
  pitch_gyro =  mpu6050.getGyroX() - pitch_sample_gyro;
  yaw_gyro =  (mpu6050.getGyroZ() * -1) - yaw_sample_gyro;

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //COMPLIMENTARY FILTER 100 = 70(of previous value) + 30(of updated value)
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  angle_roll_input = (angle_roll_input * 0.94) + ((roll_angle) * 0.06);     //roll angle
  angle_pitch_input = (angle_pitch_input * 0.94) + ((pitch_angle) * 0.06);  //pitch angle
  angle_yaw_input = (angle_yaw_input * 0.94) + ((yaw_angle) * 0.06);        //yaw angle

  gyro_roll_input = (gyro_roll_input * 0.7) + ((roll_gyro) * 0.3)  ;     //roll gyro
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((pitch_gyro) * 0.3);    //pitch gyro
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((yaw_gyro) * 0.3);          //yaw gyro


  //    Serial.print(angle_roll_input);
  //    Serial.print("\t");
  //    Serial.print(angle_pitch_input);
  //    Serial.print("\t");
  //    Serial.println(angle_yaw_input);


  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //PS3 INPUTS
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  throttle = PS4.data.analog.stick.ly;
  throttle = map(throttle, -127, 128, 0, 255);



  pid_roll_setpoint = 0;
  roll = PS4.data.analog.stick.rx;
  pid_roll_setpoint = map(roll, 127, -128, 80, -80);
  pid_roll_setpoint -= angle_roll_input;
  pid_roll_setpoint /= 2;



  pid_pitch_setpoint = 0;
  pitch = PS4.data.analog.stick.ry;
  pid_pitch_setpoint = map(pitch, -127, 128, 80, -80);
  pid_pitch_setpoint -= angle_pitch_input;
  pid_pitch_setpoint /= 2;

  pid_yaw_setpoint = 0;
  yaw = PS4.data.analog.stick.lx;
  pid_yaw_setpoint = map(yaw, 127, -128, 180, -180);
  pid_yaw_setpoint -= angle_yaw_input;
  pid_yaw_setpoint /= 2;

//      Serial.print(pid_roll_setpoint);
//      Serial.print("\t");
//      Serial.print(pid_pitch_setpoint);
//      Serial.print("\t");
//      Serial.println(pid_yaw_setpoint);






    if ( PS4.data.button.cross)
    { drone = true;
  
  
      //Reset the PID controllers for a bumpless start.
      pid_i_mem_roll = 0;
      pid_last_roll_d_error = 0;
      pid_i_mem_pitch = 0;
      pid_last_pitch_d_error = 0;
      pid_i_mem_yaw = 0;
      pid_last_yaw_d_error = 0;
    }
  
    if (PS4.data.button.triangle)
    {
      drone = false;
    }
  
    calculate_pid();
  
  
  
    if ( drone == true ) {
  
  
      lights(1);
      esc_2 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
      esc_3 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
      esc_4 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
      esc_1 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)
  
      if (esc_1 <= 10) esc_1 = 0;
      if (esc_2 <= 10) esc_2 = 0;
      if (esc_3 <= 10) esc_3 = 0;
      if (esc_4 <= 10) esc_4 = 0;
  
      if (esc_1 >= 255)esc_1 = 255;
      if (esc_2 >= 255)esc_2 = 255;
      if (esc_3 >= 255)esc_3 = 255;
      if (esc_4 >= 255)esc_4 = 255;
  
  
  
  

      ledcWrite(ledChannel1, esc_1);
      ledcWrite(ledChannel2, esc_2);
      ledcWrite(ledChannel3, esc_3);
      ledcWrite(ledChannel4, esc_4);
    }
    else {
      lights(0);
      ledcWrite(ledChannel1, 0);
      ledcWrite(ledChannel2, 0);
      ledcWrite(ledChannel3, 0);
      ledcWrite(ledChannel4, 0);
    }



}
