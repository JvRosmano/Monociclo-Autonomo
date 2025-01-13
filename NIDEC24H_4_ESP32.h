/*
 *  NIDEC24H_4_ESP32.h
 *
 *  João Vitor de M.G. Rosmaninho <jvrosmaninho@ufmg.br>
 *
 *  Version 1.0 - API with the following implemented functions:
 *  void motorInit(motor *motorToInit, int enc_a, int enc_b, int dir, int pwm, int brake, int pwm_ch);
 *  void motorControl(motor *motorToControl, int pwm);
 *	void motorSetup(motor *motorToSetup);
 *	void encoderSetup(motor *motorToInit, angleControl *control);
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
#define TIMER_BIT 8
#define BASE_FREQ 20000
// Struct motor
typedef struct motor
{
  int enc_a;  // Purple wire (Signal A)
  int enc_b;  // Orange   (Signal B)
  int dir;    // Green wire  (Forward/Reverse)
  int pwm;    // Write wire  (PWM)
  int brake;  // Yellow wire (Start/Stop)
  int pwm_ch; // PWM channel
} motor;

// Inicializar pinos dos motores.
void motorInit(motor *motorToInit, int enc_a, int enc_b, int dir, int pwm, int brake, int pwm_ch);
// Definir PWM no motor
void motorControl(motor *motorToControl, int pwm);
// Inicializar motor
void motorSetup(motor *motorToSetup);
// Inicializar encoder
void encoderSetup(motor *motorToInit, angleControl *control);

#endif