#include <SimplyAtomic.h>
#define ENCODER_A_M1 7   // amarillo
#define ENCODER_B_M1 8   // verde MOTOR BASE
#define ENCODER_A_M2 9   // amarillo
#define ENCODER_B_M2 10  // verde MOTOR reabilitador
#define PWM_M1 6         // MOTOR BASE
#define PWM_M2 11        // MOTOR reabilitador
#define Direccion_Motor_2_M1 12
#define Direccion_Motor_1_M1 13  // MOTOR BASE
#define Direccion_Motor_2_M2 14
#define Direccion_Motor_1_M2 15  // MOTOR reabilitador
int Sensor_muscular = 28;
int Sensor_giroscopio_x = 26;
int Sensor_giroscopio_y = 27;
int Muscular_signal = 0;
int angulo_mano = 0;
#define DEBUG(a) Serial.println(a);

volatile int posi1 = 0, posi2 = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;
long int resolucion1 = 14400;  // 12 vueltas X 2 X 600 de reduccion
long int posicionDeseada = 0, angulotex = 0;
long posiciong = 0;
int sentidogiro = 1;
String data = "", anguloT = "", Eslabon = "", sentido = "", Estrele = "";
int angulofinal, angulofinal2, anguloencoder = 0, anguloencoder2 = 0;
int nivel_movilidad = 0;
int angulo_mano_x = 0, angulo_mano_y = 0;
int Ax,AY;
bool leer = true;
void setup() {
  Serial.begin(9600);

  pinMode(Sensor_muscular, INPUT);
  pinMode(Sensor_giroscopio_x, INPUT);
  pinMode(Sensor_giroscopio_y, INPUT);

  pinMode(ENCODER_A_M1, INPUT);
  pinMode(ENCODER_B_M1, INPUT);
  pinMode(PWM_M1, OUTPUT);
  pinMode(Direccion_Motor_1_M1, OUTPUT);
  pinMode(Direccion_Motor_2_M1, OUTPUT);

  pinMode(ENCODER_A_M2, INPUT);
  pinMode(ENCODER_B_M2, INPUT);
  pinMode(PWM_M2, OUTPUT);
  pinMode(Direccion_Motor_1_M2, OUTPUT);
  pinMode(Direccion_Motor_2_M2, OUTPUT);
  //attachInterrupt(digitalPinToInterrupt(ENCODER_A_M1), readEncoder1, RISING);
  //attachInterrupt(digitalPinToInterrupt(ENCODER_A_M2), readEncoder2, RISING);
}

void loop() {
  if (leer == true) {
    Muscular_signal = analogRead(Sensor_muscular);
    angulo_mano_x = analogRead(Sensor_giroscopio_x);
    angulo_mano_y = analogRead(Sensor_giroscopio_y);
    Ax = map(angulo_mano_x, 0, 1024, -40, 40);
    AY = map(angulo_mano_y, 0, 1024, -40, 40);
    nivel_movilidad = map(Muscular_signal, 0, 1024, 0, 100);
    Serial.print(Ax);
    Serial.print(",");
    Serial.print(AY);
    Serial.print(",");
    Serial.print(nivel_movilidad);
    Serial.print(",");
    Serial.print(anguloencoder2);
    Serial.print(",");
    Serial.println(anguloencoder);
    delay(50);
  }
  if (Serial.available()) {
    data = Serial.readStringUntil('\n');
    DEBUG(data);
    Eslabon = data.substring(3, 4);  //extraer eslabon mover
    sentido = data.substring(5, 6);  //extraer sentido de giro
    anguloT = data.substring(0, 3);  //extraer angulo para mover
    if (sentido == "+") {
      sentidogiro = 1;
    } else if (sentido == "-") {
      sentidogiro = -1;
    }
  }
  if (Eslabon == "b")  //b =base
  {
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_M1), readEncoder1, RISING);
    angulotex = anguloT.toInt();
    posicionDeseada = (angulotex * sentidogiro) / 2;
    long int target = (posicionDeseada * resolucion1) / 360;
    // PID constants
    float kp = 1.35;       //1.22
    float kd = 0.0001445;  //0.001025
    float ki = 0.00045;    //0.0015 2° +- //0.0025 4° + // 0.0010 + 2 °;
    // diferencia de tiempo
    long currT = micros();
    float deltaT = ((float)(currT - prevT)) / (1.0e6);
    prevT = currT;
    int pos = 0;
    ATOMIC() {
      pos = posi1;
    }
    // error
    int e = pos - target;
    // derivative
    float dedt = (e - eprev) / (deltaT);

    // integral
    eintegral = eintegral + e * deltaT;

    // control signal
    float u = kp * e + kd * dedt + ki * eintegral;

    // velocidad/torque del motor
    float pwr = fabs(u);
    if (pwr > 255) {
      pwr = 255;
    }
    // motor direccion
    int dir = 1;
    if (u < 0) {
      dir = -1;
    }
    setMotor(dir, pwr, PWM_M1, Direccion_Motor_1_M1, Direccion_Motor_2_M1);
    // Error previo
    eprev = e;
    angulofinal = ((target * 360) / resolucion1) * 2;
    anguloencoder = ((posi1 * 360) / resolucion1) * 2;
    // Serial.print(angulofinal);
    //Serial.print(" ");
    //Serial.print(anguloencoder);
    // Serial.println();
    if (anguloencoder == angulofinal) {
      digitalWrite(Direccion_Motor_2_M1, LOW);
      digitalWrite(Direccion_Motor_1_M1, LOW);
    } else {
      readEncoder1();
      //delay(10);
    }
  }
  if (Eslabon == "c")  //c = efector final
  {
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_M2), readEncoder2, RISING);
    angulotex = anguloT.toInt();
    posicionDeseada = (angulotex * sentidogiro) / 2;
    long int target = (posicionDeseada * resolucion1) / 360;
    // PID constants
    float kp = 1.25;      //1.22
    float kd = 0.002425;  //0.001025
    float ki = 0.0025;    //0.0015 2° +- //0.0025 4° + // 0.0010 + 2 °;
    long currT = micros();
    float deltaT = ((float)(currT - prevT)) / (1.0e6);
    prevT = currT;
    int pos = 0;
    ATOMIC() {
      pos = posi2;
    }
    // error
    int e = pos - target;
    // derivative
    float dedt = (e - eprev) / (deltaT);

    // integral
    eintegral = eintegral + e * deltaT;

    // control signal
    float u = kp * e + kd * dedt + ki * eintegral;

    // velocidad/torque del motor
    float pwr = fabs(u);
    if (pwr > 255) {
      pwr = 255;
    }
    // motor direccion
    int dir = 1;
    if (u < 0) {
      dir = -1;
    }
    // Error previo
    eprev = e;
    // señal del motor
    angulofinal2 = ((target * 360) / resolucion1)*2;
    anguloencoder2 = ((posi2 * 360) / resolucion1)*2;
    setMotor(dir, pwr, PWM_M2, Direccion_Motor_1_M2, Direccion_Motor_2_M2);
    //if (anguloencoder2 == angulofinal2 + 2 || anguloencoder2 == angulofinal2 - 1) {
    if (anguloencoder2 == angulofinal2) {
      digitalWrite(Direccion_Motor_2_M2, LOW);
      digitalWrite(Direccion_Motor_1_M2, LOW);
      analogWrite(PWM_M2, 0);
    } else {
      readEncoder2();
      //delay(10);
    }
  } else if (Eslabon == "S") {
    leer = true;
    analogWrite(PWM_M1, 0);
    analogWrite(PWM_M2, 0);
    digitalWrite(Direccion_Motor_2_M1, LOW);
    digitalWrite(Direccion_Motor_1_M1, LOW);
    digitalWrite(Direccion_Motor_2_M2, LOW);
    digitalWrite(Direccion_Motor_1_M2, LOW);
    anguloencoder2 = 0;
    anguloencoder = 0;
    reset();
  }
}

void setMotor(int dir, int voltaje_pwm, int pwm, int direccion_Motor_1, int direccion_Motor_2) {
  analogWrite(pwm, voltaje_pwm);
  if (dir == 1) {
    digitalWrite(direccion_Motor_1, HIGH);
    digitalWrite(direccion_Motor_2, LOW);
  } else if (dir == -1) {
    digitalWrite(direccion_Motor_1, LOW);
    digitalWrite(direccion_Motor_2, HIGH);
  } else {
    digitalWrite(direccion_Motor_1, LOW);
    digitalWrite(direccion_Motor_2, LOW);
  }
}
void reset() {
  prevT = 0;
  eprev = 0;
  eintegral = 0;
}
void readEncoder1() {
  int b = digitalRead(ENCODER_B_M1);
  if (b > 0) {
    posi1++;
  } else {
    posi1--;
  }
}
void readEncoder2() {
  int b = digitalRead(ENCODER_B_M2);
  if (b > 0) {
    posi2++;
  } else {
    posi2--;
  }
}
