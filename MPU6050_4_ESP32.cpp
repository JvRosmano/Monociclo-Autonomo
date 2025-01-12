/*
 *  MPU6050_4_ESP32.c
 *
 *  João Vitor de M.G. Rosmaninho <jvrosmaninho@ufmg.br>
 *
 *  Version 1.0 - API with the following implemented functions:
 *  void LocationService_Init(UART_HandleTypeDef *huart, TIM_HandleTypeDef* htim);
 *  float LocationService_CalculateDistance(int rssi);
 *	location_t LocationService_GetLocation();
 *	uint8_t LocationService_IsInDestiny();
 *	float LocationService_GetArrivalAngle();
 *
 *  Created on 2024
 *  Institution: UFMG
 *  This API contains functions to use of some hardware resources
 *  from the MPU6050 Gyroscope/Accelerometer module.
 */

#include "MPU6050_4_ESP32.h"

// SETUP functions
void imuSetup(){
  // Initialize the MPU6050
  Wire.beginTransmission(MPU);           //begin, Send the slave adress (in this case 68)              
  Wire.write(0x6B);                      //make the reset (place a 0 into the 6B register)
  Wire.write(0);
  Wire.endTransmission(true);            //end the transmission
  //Gyro config
  Wire.beginTransmission(MPU);           //begin, Send the slave adress (in this case 68) 
  Wire.write(0x1B);                      //We want to write to the GYRO_CONFIG register (1B hex)
  // Wire.write(0x00000000);             //Set the register bits as 00000000 (250dps full scale), 00010000 (1000dps full scale)
  Wire.write(1 << 3);
  Wire.endTransmission();                //End the transmission with the gyro
  //Acc config
  Wire.beginTransmission(MPU);           //Start communication with the address found during search.
  Wire.write(0x1C);                      //We want to write to the ACCEL_CONFIG register
  Wire.write(0b00000000);                //Set the register bits as 00000000 (+/- 2g full scale range), 00010000 (+/- 8g full scale range)
  Wire.endTransmission(); 
}

// IMU function: Kalman Filter
void angleCalc(angleControl *rollControl, angleControl *pitchControl, kalman *roll_angle, kalman *pitch_angle, float Ts){
  // read IMU
  int16_t ax,ay,az,temp,gx,gy,gz;
  Wire.beginTransmission(MPU);    // IMU address: 0x68
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14);        // IMU address: 0x68
  ax = Wire.read() << 8 | Wire.read();   // X-axis value: 16384.0; 
  ay = Wire.read() << 8 | Wire.read();   // Y-axis value: 16384.0;     
  az = Wire.read() << 8 | Wire.read();   // Z-axis value: 16384.0;  
  temp = Wire.read() << 8 | Wire.read();      
  gx = Wire.read() << 8 | Wire.read();  
  gy = Wire.read() << 8 | Wire.read();  
  gz = Wire.read() << 8 | Wire.read();  
  //accelerometer angles in degrees (or rads)
  // roll
  float ax_angle = atan2(ay, sqrt(ax*ax + az*az)) * 57.3; // roll
  float gyrox =  gx / 65.5; 
  // pitch
  float ay_angle = atan2(ax, sqrt(ay*ay + az*az)) * 57.3; // pitch
  float gyroy =  gy / 65.5; 
  // begin: Kalman filter - Roll Axis (X)
  executeKalman(rollControl, roll_angle, ax_angle, gyrox, Ts);
  // begin: Kalman filter - Pitch Axis (Y)
  executeKalman(pitchControl, pitch_angle, ay_angle, gyroy, Ts);
} 

// Inicia estrutura de kalman
void kalmanInit(kalman *kalmanFilter){
  kalmanFilter->Q.angle = 0.001;
  kalmanFilter->Q.bias = 0.005;
  kalmanFilter->R_meas = 1.0;
  kalmanFilter->U.angle = 0.0;
  kalmanFilter->U.bias = 0.0;
  kalmanFilter->P[0][0] = 1.0;
  kalmanFilter->P[0][1] = 0.0;
  kalmanFilter->P[1][0] = 0.0;
  kalmanFilter->P[1][1] = 1.0;

  kalmanFilter->K[0] = 0.0;
  kalmanFilter->K[1] = 0.0;
}

void executeKalman(angleControl *control, kalman *filter, float angle, float gyroAxis, float Ts){
  filter->U.angle += (gyroAxis - filter->U.bias) * Ts;

  filter->P[0][0] += (filter->Q.angle - filter->P[0][1] - filter->P[1][0]) * Ts;
  filter->P[0][1] += -filter->P[1][1] * Ts;
  filter->P[1][0] += -filter->P[1][1] * Ts;
  filter->P[1][1] += filter->Q.bias * Ts;
  //
  filter->K[0] = filter->P[0][0] / (filter->P[0][0] + filter->R_meas);
  filter->K[1] = filter->P[1][0] / (filter->P[0][0] + filter->R_meas);  
  //
  filter->U.angle += filter->K[0] * (angle - filter->U.angle); 
  filter->U.bias += filter->K[1] * (angle - filter->U.angle);
  //
  float P00_temp = filter->P[0][0];
  float P01_temp = filter->P[0][1];

  filter->P[0][0] -= filter->K[0] * P00_temp;
  filter->P[0][1] -= filter->K[0] * P01_temp;
  filter->P[1][0] -= filter->K[1] * P00_temp;
  filter->P[1][1] -= filter->K[1] * P01_temp;
  // end: Kalman filter 

  control->body_speed = gyroAxis - filter->U.bias; // Unbiased gyro speed
}