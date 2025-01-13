/*
 *  NIDEC24H_4_ESP32.cpp
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

#include "NIDEC24H_4_ESP32.h"

void motorInit(motor *motorToInit, int enc_a, int enc_b, int dir, int pwm, int brake, int pwm_ch)
{
  // Inicializa pinos do motor
  motorToInit->enc_a = enc_a;
  motorToInit->enc_b = enc_b;
  motorToInit->dir = dir;
  motorToInit->pwm = pwm;
  motorToInit->brake = brake;
  motorToInit->pwm_ch = pwm_ch;
};

void motorControl(motor *motorToControl, int pwm)
{
  // A depender do sinal, inverte a rotação
  if (pwm < 0)
  {
    digitalWrite(motorToControl->dir, LOW);
    pwm = -pwm;
  }
  else
  {
    digitalWrite(motorToControl->dir, HIGH);
  }
  // Escreve no pwm
  ledcWrite(motorToControl->pwm_ch, int(pwm > 255 ? 255 : 255 - pwm));
};

void motorSetup(motor *motorToSetup)
{
  // Inicializa pinos e associa canais
  pinMode(motorToSetup->brake, OUTPUT);
  digitalWrite(motorToSetup->brake, HIGH);
  pinMode(motorToSetup->dir, OUTPUT);
  ledcSetup(motorToSetup->pwm_ch, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(motorToSetup->pwm, motorToSetup->pwm_ch);
  // Inicia motor desligado
  motorControl(motorToSetup, 0);
};

void encoderSetup(motor *motorToInit, angleControl *control)
{
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  control->encoder.attachFullQuad(motorToInit->enc_b, motorToInit->enc_a);
  control->encoder.clearCount();
}