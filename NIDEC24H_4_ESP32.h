/*
 *  NIDEC24H_4_ESP32.h
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
 *  from the NIDEC 24H DC Motor.
 */

#ifndef NIDEC24H_4_ESP32_H
#define NIDEC24H_4_ESP32_H
// ENCODER library based on the built in counter hardware
// #include <ESP32Encoder.h>
#include "Control.h"
#include <Arduino.h>
// NIDEC PWM config
#define TIMER_BIT   8
#define BASE_FREQ   20000
// Struct motor
typedef struct motor {
  int enc_a; // Purple wire (Signal A)
  int enc_b; // Orange   (Signal B)
  int dir; // Green wire  (Forward/Reverse) 
  int pwm; // Write wire  (PWM)
  int brake; // Yellow wire (Start/Stop)
  int pwm_ch; // PWM channel
} motor;

// Initialize motor's pins.
void motorInit(motor *motorToInit, int enc_a, int enc_b, int dir, int pwm, int brake, int pwm_ch);
// Set pwm in the motor
void motorControl(motor *motorToControl, int pwm);
// Setup the motor
void motorSetup(motor *motorToSetup);
// Setup the encoder
void encoderSetup(motor *motorToInit, angleControl *control);

 #endif