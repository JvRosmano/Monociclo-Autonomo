/*
 *  Control.cpp
 *
 *  João Vitor de M.G. Rosmaninho <jvrosmaninho@ufmg.br>
 *
 *  Version 1.0 - API with the following implemented functions:
 *  void controlInit(angleControl *control, float K1, float K2, float K3, float K4, ESP32Encoder encoder);
 *  void resetControl(angleControl *control);
 *	int executeControl(angleControl *control, float Ts);
 *
 *  Created on 2024
 *  Institution: UFMG
 *  This API contains functions to implement a control structure and a control law.
 */

#include "Control.h"

void controlInit(angleControl *control, float K1, float K2, float K3, float K4, ESP32Encoder encoder)
{
  control->K1 = K1;
  control->K2 = K2;
  control->K3 = K3;
  control->K4 = K4;
  control->body_position = 0;
  control->body_speed = 0;
  control->wheel_speed = 0;
  control->wheel_position = 0;
  control->encoder = encoder;
}

void resetControl(angleControl *control)
{
  // Nula valores
  control->body_position = 0.0;
  control->encoder.clearCount();
  control->wheel_position = 0.0;
}

// Realiza o controle para um determinado angulo
int executeControl(angleControl *control, float Ts)
{
  // Integra a velocidade para obter posição
  control->body_position += control->body_speed * Ts;
  // Obtém rotação da roda através do encoder
  control->wheel_speed = control->encoder.getCount(); // reaction wheel speed
  control->encoder.clearCount();
  // Integra a velocidade para obter posição
  control->wheel_position += control->wheel_speed;
  // Aplica a lei de controle e limita ao valor do PWM.
  return constrain(control->K1 * control->body_position + control->K2 * control->body_speed +
                       control->K3 * control->wheel_speed + control->K4 * control->wheel_position,
                   -255, 255);
}