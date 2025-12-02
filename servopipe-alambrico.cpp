#include <ESP32Servo.h>

Servo servo13; // Índice
Servo servo12; // Índice + Medio
Servo servo14; // Índice + Medio + Anular

const int pinServo13 = 13;
const int pinServo12 = 12;
const int pinServo14 = 14;

int pos13 = 90;
int pos12 = 90;
int pos14 = 90;

char comando = 'S';
unsigned long ultimoMovimiento = 0;

const int periodoRefresco = 8;

const int paso = 2; 

void setup() {
  Serial.begin(115200);
  
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servo13.setPeriodHertz(50);
  servo12.setPeriodHertz(50);
  servo14.setPeriodHertz(50);

  servo13.attach(pinServo13, 500, 2400);
  servo12.attach(pinServo12, 500, 2400);
  servo14.attach(pinServo14, 500, 2400);

  servo13.write(90);
  servo12.write(90);
  servo14.write(90);
}

void loop() {
  if (Serial.available() > 0) {
    comando = Serial.read();
    while(Serial.available() > 0) Serial.read(); 
  }

  // Lógica de movimiento por tiempo
  if (millis() - ultimoMovimiento > periodoRefresco) {
    
    switch (comando) {
      // --- SERVO 13 ---
      case 'A': pos13 += paso; break; 
      case 'B': pos13 -= paso; break; 

      // --- SERVO 12 ---
      case 'C': pos12 += paso; break; 
      case 'D': pos12 -= paso; break; 
      
      // --- SERVO 14 ---
      case 'E': pos14 += paso; break; 
      case 'F': pos14 -= paso; break; 

      // --- RESET (MEÑIQUE) ---
      case 'R': 
        pos13 = 90;
        pos12 = 90;
        pos14 = 90;
        break;
        
      case 'S': break; // Stop
    }

    // Restricciones físicas
    pos13 = constrain(pos13, 0, 180);
    pos12 = constrain(pos12, 0, 180);
    pos14 = constrain(pos14, 0, 180);

    // Escribir a los motores
    servo13.write(pos13);
    servo12.write(pos12);
    servo14.write(pos14);

    ultimoMovimiento = millis();
  }
}
