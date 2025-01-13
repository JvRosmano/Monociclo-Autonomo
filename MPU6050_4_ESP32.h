/*
 *  MPU6050_4_ESP32.h
 *
 *  João Vitor de M.G. Rosmaninho <jvrosmaninho@ufmg.br>
 *
 *  Version 1.0 - API with the following implemented functions:
 *  void imuSetup();
 *  void angleCalc(angleControl *rollControl, angleControl *pitchControl, kalman *roll_angle, kalman *pitch_angle, float Ts);
 *	void kalmanInit(kalman *kalmanFilter);
 *	void executeKalman(angleControl *control, kalman *filter, float angle, float gyroAxis, float Ts);
 *
 *  Created on 2024
 *  Institution: UFMG
 *  This API contains functions to use of some hardware resources
 *  from the MPU6050 Gyroscope/Accelerometer module.
 */

#ifndef MPU6050_4_ESP32_H
#define MPU6050_4_ESP32_H
#include "Wire.h"
#include "Control.h"
#include <Arduino.h>
// IMU I2C address
#define MPU 0x68

// Struct state
typedef struct statesToControl
{
  float angle;
  float bias;
} state;

// Struct kalman
typedef struct kalmanFilter
{
  state Q; // Error states
  float R_meas;
  state U; // States
  float P[2][2];
  float K[2];
} kalman;

// Inicializa imu
void imuSetup();
// Calcula ângulos
void angleCalc(angleControl *rollControl, angleControl *pitchControl, kalman *roll_angle, kalman *pitch_angle, float Ts);
// Inicia estrutura de kalman
void kalmanInit(kalman *kalmanFilter);
// Realiza cálculo do ângulo por kalman
void executeKalman(angleControl *control, kalman *filter, float angle, float gyroAxis, float Ts);
#endif