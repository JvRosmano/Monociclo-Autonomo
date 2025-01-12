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

#include "Control.h"

void controlInit(angleControl *control, float K1, float K2, float K3, float K4, ESP32Encoder encoder){
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

void resetControl(angleControl *control){
  // Nula valores
  control->body_position = 0.0;
  control->encoder.clearCount();
  control->wheel_position = 0.0; 
}

int executeControl(angleControl *control, float Ts){
    // Realiza o controle para um determinado angulo
    control->body_position += control->body_speed * Ts;  
    control->wheel_speed = control->encoder.getCount(); //reaction wheel speed
    control->encoder.clearCount();
    control->wheel_position += control->wheel_speed;           //reaction wheel position
  return constrain(control->K1 * control->body_position + control->K2 * control->body_speed + 
  control->K3 * control->wheel_speed + control->K4 * control->wheel_position, -255, 255);
}