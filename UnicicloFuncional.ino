// I2C libray communication
#include <Wire.h>

// ENCODER library based on the built in counter hardware
#include <ESP32Encoder.h>

// ESP32 BLUE LED pin
#define INTERNAL_LED 2

// IMU I2C address
#define MPU   0x68

// NIDEC PWM config
#define TIMER_BIT   8
#define BASE_FREQ   20000

// Struct motor
typedef struct motor {
  int brake; // Yellow wire (Start/Stop)
  int pwm; // Write wire  (PWM)
  int dir; // Green wire  (Forward/Reverse) 
  int enc_a; // Purple wire (Signal A)
  int enc_b; // Orange   (Signal B)
  int pwm_ch; // PWM channel
} motor;

// Inicialização dos motores
// motor motor1, motor2, motor3;
motor motor1 = {
  brake: 13, // Yellow wire (Start/Stop)
  pwm: 5, // Write wire  (PWM)
  dir: 23, // Green wire  (Forward/Reverse) 
  enc_a: 19, // Purple wire (Signal A)
  enc_b: 18, // Orange   (Signal B)
  pwm_ch: 1 // PWM channel
};

motor motor2 = {
  brake: 26, // Yellow wire (Start/Stop)
  pwm: 4, // Write wire  (PWM)
  dir: 25, // Green wire  (Forward/Reverse) 
  enc_a: 35, // Purple wire (Signal A)
  enc_b: 34, // Orange   (Signal B)
  pwm_ch: 0 // PWM channel
};

motor motor3 = {
  brake: 17, // Yellow wire (Start/Stop)
  pwm: 16, // Write wire  (PWM)
  dir: 27, // Green wire  (Forward/Reverse) 
  enc_a: 14, // Purple wire (Signal A)
  enc_b: 39, // Orange   (Signal B)
  pwm_ch: 2 // PWM channel
};

// Encoder var
ESP32Encoder NIDEC1_ENC, NIDEC2_ENC;

// Montando a estrutura para o filtro de kalman
typedef struct statesToControl {
  float angle;
  float bias;
} state;

typedef struct kalmanFilter {
  state Q; // Error states
  float R_meas;
  state U; // States
  float P[2][2];
  float K[2];
} kalman;

// Filtro de kalman para ângulo de roll
kalman roll_angle = {
  Q: {
    angle: 0.001, // Angular data confidence
    bias: 0.005 // Angular velocity data confidence
  },
  R_meas: 1.0,
  U: {
    angle: 0.0, // angle
    bias: 0.0 // bias
  },
  P: {{ 1, 0 }, { 0, 1 }},
  K: {0, 0}
};

// Filtro de kalman para ângulo de pitch
kalman pitch_angle = {
  Q: {
    angle: 0.001, // Angular data confidence
    bias: 0.005 // Angular velocity data confidence
  },
  R_meas: 1.0,
  U: {
    angle: 0.0, // angle
    bias: 0.0 // bias
  },
  P: {{ 1, 0 }, { 0, 1 }},
  K: {0, 0}
};

// Estrutura de controle
typedef struct controlStructure {
  float K1;
  float K2;
  float K3;
  float K4;
  float K5;
  float deslocamento;
  float body_position;
  float body_speed;
  int wheel_speed;
  int wheel_position;
  ESP32Encoder encoder;
} angleControl;

// Definições para roll e pitch
angleControl rollControl = {
  K1: 90.0,
  K2: 0.5,
  K3: 0.5,
  K4: 0.025,
  K5: 0.0,
  deslocamento: 0.0,
  body_position: 0.0,
  body_speed: 0.0,
  wheel_speed: 0,
  wheel_position: 0
};

angleControl pitchControl = {
  K1: 11,
  K2: 0.1,
  K3: 0.31,
  K4: 0.06,
  K5: 0.0,
  body_position: 0.0,
  body_speed: 0.0,
  wheel_speed: 0,
  wheel_position: 0,
  encoder: NIDEC2_ENC
};

float Ts = 0.1, currentT = 0.0, previousT = 0.0;        // Elapsed time in loop() function

// modo = 0, ambos motores
// modo = 1, só motores de reação
// modo = 2, só motor central
int modo = 0;

// put your setup code here, to run once:
void setup() { 
  Wire.begin();
  Serial.begin(115200); 
  // Inicializa motor
  motors_setup();
  // Inicializa imu
  imu_setup();
  // Acende led para indicar período de estabilização
  pinMode(INTERNAL_LED, OUTPUT);
  digitalWrite(INTERNAL_LED, HIGH);  // Turn on red led
  delay(1000);                      // Wait for the system to stabilize
  for (int i=1; i<= 400; i++){      // Wait for the Kalman filter stabilize
    angle_calc();
    delay(5);
  }
  digitalWrite(INTERNAL_LED, LOW);  
}
// put your main code here, to run repeatedly:
void loop() {
  
  currentT = millis();
  if ((currentT - previousT)/1000.0 >= Ts) {
    previousT = currentT;
    
    angle_calc();
    // Serial.println(pitch_angle.U.angle);
    if (
      abs(roll_angle.U.angle) < 10
    && 
    abs(pitch_angle.U.angle) < 20
    ){

      // Controle Roll
      if(modo == 0 || modo == 1){
        digitalWrite(motor1.brake, HIGH);
        digitalWrite(motor2.brake, HIGH);
        
        int pwmX = executeControl(rollControl);
        motor_control(motor1, -pwmX);
        motor_control(motor2, pwmX);
      }
      // Controle Pitch
      if(modo == 0 || modo == 2){
        digitalWrite(motor3.brake, HIGH);

        int pwmY = executeControl(pitchControl);
        motor_control(motor3, -pwmY);
      }
    } 
    else { 
      digitalWrite(INTERNAL_LED,HIGH);  
      motor_control(motor1, 0); // stop reaction wheel
      motor_control(motor2, 0); // stop reaction wheel 
      motor_control(motor3, 0); // stop reaction wheel 
      delay(5000);       // Tempo de aguardo
      digitalWrite(INTERNAL_LED, LOW);
      for (int i=1; i<= 400; i++){ //Wait for the Kalman Filter stabilize
        angle_calc();
        delay(5);
      }
      previousT = millis();
      if(modo == 0 || modo == 1){
        reset_control(rollControl);
      }
      if(modo == 0 || modo == 2){
        reset_control(pitchControl);
      }
    }
  }   

}


// SETUP functions
void imu_setup(){
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

void motors_setup(){
  // Inicializa motores e encoders
  motor_init(motor1);
  motor_init(motor2);
  encoder_setup(motor1, rollControl);
  motor_init(motor3);
  encoder_setup(motor3, pitchControl);
}

void motor_init(motor motorToInit){
  pinMode(motorToInit.brake, OUTPUT);
  digitalWrite(motorToInit.brake, HIGH);
  
  pinMode(motorToInit.dir, OUTPUT);
  ledcSetup(motorToInit.pwm_ch, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(motorToInit.pwm, motorToInit.pwm_ch);
  motor_control(motorToInit, 0);
}

void encoder_setup(motor motorToInit, angleControl &control){
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
	control.encoder.attachFullQuad(motorToInit.enc_b, motorToInit.enc_a);
  control.encoder.clearCount();
}
// IMU function: Kalman Filter
void angle_calc(){
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
  executeKalman(rollControl, roll_angle, ax_angle, gyrox);
  // begin: Kalman filter - Pitch Axis (Y)
  executeKalman(pitchControl, pitch_angle, ay_angle, gyroy);
}  

// Controlar motor
void motor_control(motor motorToControl, int sp) {
  // A depender do sinal, inverte a rotação
  if (sp < 0) {
    digitalWrite(motorToControl.dir, LOW);
    sp = -sp;
  } else {
    digitalWrite(motorToControl.dir, HIGH);
  }
  // Escreve no pwm
  ledcWrite(motorToControl.pwm_ch, int(sp > 255 ? 255 : 255 - sp));
}

// Resetar controle
void reset_control(angleControl &control){
  // Nula valores
  control.body_position = 0.0;
  control.encoder.clearCount();
  control.wheel_position = 0.0;  
}

void executeKalman(angleControl &control, kalman &filter, float angle, float gyroAxis){
  filter.U.angle += (gyroAxis - filter.U.bias) * Ts;

  filter.P[0][0] += (filter.Q.angle - filter.P[0][1] - filter.P[1][0]) * Ts;
  filter.P[0][1] += -filter.P[1][1] * Ts;
  filter.P[1][0] += -filter.P[1][1] * Ts;
  filter.P[1][1] += filter.Q.bias * Ts;
  //
  filter.K[0] = filter.P[0][0] / (filter.P[0][0] + filter.R_meas);
  filter.K[1] = filter.P[1][0] / (filter.P[0][0] + filter.R_meas);  
  //
  filter.U.angle += filter.K[0] * (angle - filter.U.angle); 
  filter.U.bias += filter.K[1] * (angle - filter.U.angle);
  //
  float P00_temp = filter.P[0][0];
  float P01_temp = filter.P[0][1];

  filter.P[0][0] -= filter.K[0] * P00_temp;
  filter.P[0][1] -= filter.K[0] * P01_temp;
  filter.P[1][0] -= filter.K[1] * P00_temp;
  filter.P[1][1] -= filter.K[1] * P01_temp;
  // end: Kalman filter 

  control.body_speed = gyroAxis - filter.U.bias; // Unbiased gyro speed
}

int executeControl(angleControl &control){
    // Realiza o controle para um determinado angulo
    control.deslocamento += control.body_position * Ts;
    control.body_position += control.body_speed * Ts;  
    control.wheel_speed = control.encoder.getCount(); //reaction wheel speed
    control.encoder.clearCount();
    control.wheel_position += control.wheel_speed;           //reaction wheel position
  return constrain(control.K1 * control.body_position + control.K2 * control.body_speed + 
  control.K3 * control.wheel_speed + control.K4 * control.wheel_position + control.deslocamento*control.K5, -255, 255);
}