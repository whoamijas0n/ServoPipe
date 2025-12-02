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
