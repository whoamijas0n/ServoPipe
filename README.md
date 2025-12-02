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
  <img src="img/serv.png" alt="Imagen de el circuito" width="650">
</p>

### Importante

Nunca alimentes los servos directamente desde el pin 3.3V del ESP32. Usa siempre una fuente externa y unifica las tierras (GND) del ESP32 y las baterías.

## Requisitos de Software

* Python: Versión 3.8 a 3.11 (Recomendado para compatibilidad con MediaPipe).

* Arduino IDE: Para cargar el código C++ al ESP32.

* Librerías necesarias: ESP32Servo de Kevin Harrington.

* Webcam: Funcional.

  
