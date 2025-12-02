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

### Código ESP32 (C++)

Sube este código al ESP32 usando Arduino IDE.

* Asegúrate de que el Serial.begin coincida con el de Python (115200).

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

### Script Python

Ejecuta este script en tu PC.

* Edita la variable PUERTO_SERIAL (ej. /dev/ttyUSB0 en Linux o COM3 en Windows).

```python
import cv2
import mediapipe as mp
import serial
import time

# PUERTO SERIAL
PUERTO_SERIAL = '/dev/ttyUSB0' 
BAUD_RATE = 115200

print(f"Iniciando en Arch Linux...")

try:
    esp32 = serial.Serial(PUERTO_SERIAL, BAUD_RATE, timeout=0.05) # Timeout muy bajo para velocidad
    time.sleep(2)
    print(f"ESP32 Conectado en {PUERTO_SERIAL}")
except Exception as e:
    print(f"MODO SIMULACIÓN (Error Serial: {e})")
    esp32 = None

mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

def contar_dedos(hand_landmarks):
    dedos = []
    tips = [8, 12, 16, 20] 
    
    # Pulgar
    if hand_landmarks.landmark[4].x < hand_landmarks.landmark[3].x: 
        dedos.append(1)
    else:
        dedos.append(0)

    # Resto de dedos
    for tip in tips:
        if hand_landmarks.landmark[tip].y < hand_landmarks.landmark[tip - 2].y:
            dedos.append(1)
        else:
            dedos.append(0)
    return dedos

# INICIO DE CÁMARA (OPTIMIZADO PARA LINUX) 
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    print("Error: No se detecta la cámara. Prueba instalar 'v4l-utils'")
    exit()

with mp_hands.Hands(model_complexity=0, max_num_hands=1, min_detection_confidence=0.7) as hands:
    while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("Saltando frame vacío...")
            continue

        # Espejo y Color
        image = cv2.flip(image, 1)
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # Procesamiento
        results = hands.process(image_rgb)

        comando = 'S' 
        texto_pantalla = "STOP"
        color_texto = (0, 0, 255)

        if results.multi_hand_landmarks:
            for idx, hand_landmarks in enumerate(results.multi_hand_landmarks):
                mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                
                label = results.multi_handedness[idx].classification[0].label
                d = contar_dedos(hand_landmarks) 

                # LÓGICA DE GESTOS
                
                # RESET (SOLO MEÑIQUE) -> [0, 0, 0, 0, 1]
                if d[1]==0 and d[2]==0 and d[3]==0 and d[4]==1:
                    comando = 'R'
                    texto_pantalla = "RESET (MENIQUE)"
                    color_texto = (0, 0, 255)

                # TRES DEDOS
                elif d[1]==1 and d[2]==1 and d[3]==1:
                    if label == 'Right':
                        comando = 'E'
                        texto_pantalla = "DER: 3 Dedos"
                    else:
                        comando = 'F'
                        texto_pantalla = "IZQ: 3 Dedos"
                    color_texto = (255, 255, 0)

                # DOS DEDOS
                elif d[1]==1 and d[2]==1:
                    if label == 'Right':
                        comando = 'C'
                        texto_pantalla = "DER: 2 Dedos"
                    else:
                        comando = 'D'
                        texto_pantalla = "IZQ: 2 Dedos"
                    color_texto = (0, 255, 0)

                # UN DEDO
                elif d[1]==1:
                    if label == 'Right':
                        comando = 'A'
                        texto_pantalla = "DER: 1 Dedo"
                    else:
                        comando = 'B'
                        texto_pantalla = "IZQ: 1 Dedo"
                    color_texto = (0, 255, 0)

        # Enviar comando
        if esp32:
            try:
                esp32.write(comando.encode())
            except: pass

        cv2.putText(image, texto_pantalla, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color_texto, 2)
        cv2.imshow('Control ArchLinux', image)
        
        # Salir con ESC
        if cv2.waitKey(1) & 0xFF == 27:
            break

cap.release()
cv2.destroyAllWindows()
if esp32: esp32.close()
```
### Modo Inalámbrico (WiFi)

Permite libertad de movimiento utilizando el protocolo UDP para envío rápido de datos.

### Código ESP32 (C++)

Primero tienes que subir este código al ESP32 y abrir el Monitor Serial para obtener la DIRECCIÓN IP.

* Configuración: Edita las variables ssid y password con tus credenciales WiFi.

```c++
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// TUS DATOS DE WI-FI
const char* ssid = "SSID DE LA RED";
const char* password = "CONTRASEÑA DE LA RED";

// Configuración UDP
WiFiUDP udp;
unsigned int localPort = 4210; // Puerto escucha
char packetBuffer[255]; // Buffer para guardar datos entrantes

// SERVOS 
Servo servo13; 
Servo servo12; 
Servo servo14; 

const int pinServo13 = 13;
const int pinServo12 = 12;
const int pinServo14 = 14;

// Posiciones
int pos13 = 90;
int pos12 = 90;
int pos14 = 90;

// Variables de suavizado
char comando = 'S';
unsigned long ultimoMovimiento = 0;
const int periodoRefresco = 8; 
const int paso = 2; 

void setup() {
  Serial.begin(115200);

  // Conexión Wi-Fi
  Serial.print("Conectando a ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.println("WiFi conectado!");
  Serial.print("MI DIRECCION IP ES: ");
  Serial.println(WiFi.localIP()); // <--- ¡ANOTA ESTE NUMERO!
  
  // Iniciar UDP
  udp.begin(localPort);
  Serial.printf("Escuchando en el puerto UDP %d\n", localPort);

  // Configurar Servos
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
  // RECEPCIÓN INALÁMBRICA (UDP)
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
    comando = packetBuffer[0];
  }

  // CONTROL DE MOVIMIENTOS
  if (millis() - ultimoMovimiento > periodoRefresco) {
    
    switch (comando) {
      case 'A': pos13 += paso; break; 
      case 'B': pos13 -= paso; break; 
      case 'C': pos12 += paso; break; 
      case 'D': pos12 -= paso; break; 
      case 'E': pos14 += paso; break; 
      case 'F': pos14 -= paso; break; 

      case 'R': 
        pos13 = 90; pos12 = 90; pos14 = 90;
        break;
        
      case 'S': break; 
    }

    pos13 = constrain(pos13, 0, 180);
    pos12 = constrain(pos12, 0, 180);
    pos14 = constrain(pos14, 0, 180);

    servo13.write(pos13);
    servo12.write(pos12);
    servo14.write(pos14);

    ultimoMovimiento = millis();
  }
}
```
### Script Python

Ejecuta este script en tu PC.

* Importante: Debes pegar la IP que te mostró el ESP32 en la variable ESP32_IP.

```python
import cv2
import mediapipe as mp
import socket  # <--- Libreria para Internet
import time

# --- CONFIGURACIÓN INALÁMBRICA ---
# Pon aquí la IP que te dió el Monitor Serial de Arduino
ESP32_IP = "192.168.1.XX"  # <--- ¡EDITA ESTO!
ESP32_PORT = 4210

print(f"Apuntando a ESP32 en {ESP32_IP}:{ESP32_PORT}")

# Crear el socket UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

def contar_dedos(hand_landmarks):
    dedos = []
    tips = [8, 12, 16, 20] 
    if hand_landmarks.landmark[4].x < hand_landmarks.landmark[3].x: 
        dedos.append(1)
    else:
        dedos.append(0)
    for tip in tips:
        if hand_landmarks.landmark[tip].y < hand_landmarks.landmark[tip - 2].y:
            dedos.append(1)
        else:
            dedos.append(0)
    return dedos

# Configuración de Cámara (V4L2 para Arch)
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

with mp_hands.Hands(model_complexity=0, max_num_hands=1) as hands:
    while cap.isOpened():
        success, image = cap.read()
        if not success: continue

        image = cv2.flip(image, 1)
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = hands.process(image_rgb)

        comando = 'S' 
        texto_pantalla = "STOP"
        color_texto = (0, 0, 255)

        if results.multi_hand_landmarks:
            for idx, hand_landmarks in enumerate(results.multi_hand_landmarks):
                mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                
                label = results.multi_handedness[idx].classification[0].label
                d = contar_dedos(hand_landmarks) 

                # --- LÓGICA DE GESTOS (Misma lógica) ---
                if d[1]==0 and d[2]==0 and d[3]==0 and d[4]==1:
                    comando = 'R'
                    texto_pantalla = "RESET (MENIQUE)"
                    color_texto = (0, 0, 255)
                elif d[1]==1 and d[2]==1 and d[3]==1:
                    comando = 'E' if label == 'Right' else 'F'
                    texto_pantalla = "3 Dedos"
                    color_texto = (255, 255, 0)
                elif d[1]==1 and d[2]==1:
                    comando = 'C' if label == 'Right' else 'D'
                    texto_pantalla = "2 Dedos"
                    color_texto = (0, 255, 0)
                elif d[1]==1:
                    comando = 'A' if label == 'Right' else 'B'
                    texto_pantalla = "1 Dedo"
                    color_texto = (0, 255, 0)

        # --- ENVIAR POR WI-FI ---
        # Enviamos la letra al ESP32 por UDP
        try:
            sock.sendto(comando.encode(), (ESP32_IP, ESP32_PORT))
        except Exception as e:
            print(f"Error de red: {e}")

        cv2.putText(image, f"WiFi: {texto_pantalla}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color_texto, 2)
        cv2.imshow('Control Inalambrico', image)
        
        if cv2.waitKey(1) & 0xFF == 27:
            break

cap.release()
cv2.destroyAllWindows()
```
## Guía de Gestos
```Plaintext
+-------------------------+---------------------------+-----------------+
|          GESTO          |          ACCIÓN           |  SERVO / GPIO   |
+-------------------------+---------------------------+-----------------+
| Solo Indice             | Mover Servo 1             | GPIO 13         |
+-------------------------+---------------------------+-----------------+
| Indice + Medio          | Mover Servo 2             | GPIO 12         |
+-------------------------+---------------------------+-----------------+
| Indice + Medio + Anular | Mover Servo 3             | GPIO 14         |
+-------------------------+---------------------------+-----------------+
| Solo Meñique            | RESET (Todos al centro)   | Todos (90 deg)  |
+-------------------------+---------------------------+-----------------+
| Puño Cerrado            | STOP (Pausar movimiento)  | Ninguno         |
+-------------------------+---------------------------+-----------------+
| Mano Derecha            | Giro sentido horario (CW) | -               |
+-------------------------+---------------------------+-----------------+
| Mano Izquierda          | Giro anti-horario (CCW)   | -               |
+-------------------------+---------------------------+-----------------+
```
## Ejemplo de Aplicación: Grúa Robótica

Este firmware está diseñado para controlar una Grúa Robótica de 3 Ejes. La lógica de control gestual permite operar la maquinaria de forma intuitiva sin tocar ningún controlador físico.

## Mapeo de Movimientos

Cada servo cumple una función mecánica específica dentro de la estructura de la grúa:

```Plaintext
+---------------------+------------------+--------------------------+-----------------------------+-----------------------------+
| COMPONENTE MECANICO |   SERVO (GPIO)   |   GESTO DE ACTIVACION    |   ACCION MANO DERECHA (CW)  |  ACCION MANO IZQUIERDA (CCW)|
+---------------------+------------------+--------------------------+-----------------------------+-----------------------------+
| Base Rotatoria      | Servo 1 (Pin 13) | 1 Dedo (Indice)          | Gira la base a la Derecha   | Gira la base a la Izquierda |
+---------------------+------------------+--------------------------+-----------------------------+-----------------------------+
| Brazo / Elevacion   | Servo 2 (Pin 12) | 2 Dedos (Indice+Medio)   | Mueve brazo hacia Arriba    | Mueve brazo hacia Abajo     |
+---------------------+------------------+--------------------------+-----------------------------+-----------------------------+
| Pinza (Gripper)     | Servo 3 (Pin 14) | 3 Dedos (+ Anular)       | Cierra la pinza (Agarrar)   | Abre la pinza (Soltar)      |
+---------------------+------------------+--------------------------+-----------------------------+-----------------------------+
```

## Modos de Control

* Pausa (Puño Cerrado ): Al cerrar el puño, los motores se frenan en su posición exacta actual. Esto es ideal para mantener el objeto suspendido en el aire mientras se mueve la base.

* Reincio (Meñique): Al levantar solo el meñique, la grúa vuelve automáticamente a su posición central (90°), alineando la base y soltando la carga por seguridad.

