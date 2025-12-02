# ServoPipe

## Control de Servos con Visión Artificial (ESP32 + MediaPipe)

Este proyecto implementa un sistema de control gestual para manos robóticas o mecanismos articulados. Utiliza Python (MediaPipe) para detectar gestos de la mano a través de una webcam y envía comandos a un microcontrolador ESP32, el cual controla múltiples servomotores.

El sistema soporta dos modos de operación:

* **Modo Alámbrico:** Comunicación Serial (USB) de baja latencia.

* **Modo Inalámbrico:** Comunicación vía WiFi (Protocolo UDP).
* 
## Hardware Necesario
* Microcontrolador: ESP32 (DevKit V1/V4 recomendado).

* Actuadores: 3x Microservos SG90 (o MG90S).

### Alimentación:

* Portapilas para 4 baterías AA (6V total).

* 1x Capacitor Electrolítico (1000uF / 16V) para estabilidad.

* 1x Interruptor (Switch).

## Diagrama del Circuito

Esquema de conexión utilizando alimentación externa para proteger el ESP32.

<p align="center">
  <img src="img/serv1.png" alt="Imagen de el circuito" width="550">
</p>

### Importante

Nunca alimentes los servos directamente desde el pin 3.3V del ESP32. Usa siempre una fuente externa y unifica las tierras (GND) del ESP32 y las baterías.

## Requisitos de Software

* Python: Versión 3.8 a 3.11 (Recomendado para compatibilidad con MediaPipe).

* Arduino IDE: Para cargar el código C++ al ESP32.

* Librerías necesarias: ESP32Servo de Kevin Harrington.

* Webcam: Funcional.

## Instalación y Entorno Virtual

Para evitar conflictos con las librerías del sistema (especialmente en Arch Linux, Debian 12+, o macOS), se recomienda usar un entorno virtual.

### 1. Clonar el repositorio
```bash
git clone https://github.com/whoamijas0n/ServoPipe.git
cd ServoPipe
```
### 2. Crear entorno virtual (venv)
```bash
python -m venv venv
```
### 3. Activar el entorno

### En Linux / MacOS:
```bash
source venv/bin/activate
```
### En Windows:
```bash
venv\Scripts\activate
```
### 4. Instalar dependencias
```bash
pip install opencv-python mediapipe pyserial
```
## Modos de uso

### Modo Alámbrico (USB)

Ideal para pruebas en escritorio con latencia cero.

### * Código ESP32 (C++)

Sube este código a tu placa usando Arduino IDE.

```c++
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
  // Limpiamos buffer agresivamente para evitar LAG serial
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
```
