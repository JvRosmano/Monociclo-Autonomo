// ENCODER library based on the built in counter hardware
#include <ESP32Encoder.h>
#include "NIDEC24H_4_ESP32.h"
#include "MPU6050_4_ESP32.h"
#include "Control.h"
#include "Wire.h"
// ESP32 BLUE LED pin
#define INTERNAL_LED 2
// Inicialização motores
motor motor1, motor2, motor3;
// Inicialização estruturas de controle
angleControl pitchControl, rollControl;
// Inicialização kalman
kalman pitch_angle, roll_angle;
// Encoder var
ESP32Encoder NIDEC1_ENC, NIDEC2_ENC;
float Ts = 0.1, currentT = 0.0, previousT = 0.0; // Elapsed time in loop() function

// modo = 0, ambos motores
// modo = 1, só motores de reação
// modo = 2, só motor central
int modo = 2;

// put your setup code here, to run once:
void setup()
{
  // Inicializa comunicação
  Wire.begin();
  Serial.begin(115200);
  // Setup motores
  motorInit(&motor1, 13, 5, 23, 19, 18, 1);
  motorSetup(&motor1);
  motorInit(&motor2, 2, 4, 32, 33, 15, 0);
  motorSetup(&motor2);
  motorInit(&motor3, 25, 17, 16, 27, 14, 2);
  motorSetup(&motor3);
  // Inicialização controle
  controlInit(&rollControl, 70, 0.5, 0.3, 0.003, NIDEC1_ENC);
  controlInit(&pitchControl, 11, 0.1, 0.31, 0.06, NIDEC2_ENC);
  // Inicialização encoders
  encoderSetup(&motor1, &rollControl);
  encoderSetup(&motor3, &pitchControl);
  // Inicialização kalman
  kalmanInit(&pitch_angle);
  kalmanInit(&roll_angle);
  // // Inicializa imu
  imuSetup();
  // Acende led para indicar período de estabilização
  pinMode(INTERNAL_LED, OUTPUT);
  digitalWrite(INTERNAL_LED, HIGH);
  // Espera estabilização do sistema
  delay(1000);
  // Aguarda estabilização da posição angular
  for (int i = 1; i <= 400; i++)
  {
    angleCalc(&rollControl, &pitchControl, &roll_angle, &pitch_angle, Ts);
    delay(5);
  }
  digitalWrite(INTERNAL_LED, LOW);
}
// put your main code here, to run repeatedly:
void loop()
{
  currentT = millis();
  if ((currentT - previousT) / 1000.0 >= Ts)
  {
    previousT = currentT;

    angleCalc(&rollControl, &pitchControl, &roll_angle, &pitch_angle, Ts);
    if (abs(roll_angle.U.angle) < 10 && abs(pitch_angle.U.angle) < 20)
    {
      // Controle Roll
      if (modo == 0 || modo == 1)
      {
        digitalWrite(motor1.brake, HIGH);
        digitalWrite(motor2.brake, HIGH);

        int pwmX = executeControl(&rollControl, Ts);
        motorControl(&motor1, pwmX);
        motorControl(&motor2, -pwmX);
      }
      // Controle Pitch
      if (modo == 0 || modo == 2)
      {
        digitalWrite(motor3.brake, HIGH);

        int pwmY = executeControl(&pitchControl, Ts);
        motorControl(&motor3, pwmY);
      }
    }
    else
    {
      digitalWrite(INTERNAL_LED, HIGH);
      motorControl(&motor1, 0); // stop reaction wheel
      motorControl(&motor2, 0); // stop reaction wheel
      motorControl(&motor3, 0); // stop reaction wheel
      delay(5000);              // Tempo de aguardo
      digitalWrite(INTERNAL_LED, LOW);
      for (int i = 1; i <= 400; i++)
      { // Wait for the Kalman Filter stabilize
        angleCalc(&rollControl, &pitchControl, &roll_angle, &pitch_angle, Ts);
        delay(5);
      }
      previousT = millis();
      if (modo == 0 || modo == 1)
      {
        resetControl(&rollControl);
      }
      if (modo == 0 || modo == 2)
      {
        resetControl(&pitchControl);
      }
    }
  }
}