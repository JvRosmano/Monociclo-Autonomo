/*
 *  Control.h
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

#ifndef CONTROL_H
#define CONTROL_H

#include <ESP32Encoder.h>
#include <Arduino.h>
// Estrutura de controle
typedef struct controlStructure {
  float K1;
  float K2;
  float K3;
  float K4;
  float body_position;
  float body_speed;
  int wheel_speed;
  int wheel_position;
  ESP32Encoder encoder;
} angleControl;

// Inicializa estrutura de controle
void controlInit(angleControl *control, float K1, float K2, float K3, float K4, ESP32Encoder encoder);
// Reseta controle
void resetControl(angleControl *control);
// Executa lei de controle
int executeControl(angleControl *control, float Ts);
#endif