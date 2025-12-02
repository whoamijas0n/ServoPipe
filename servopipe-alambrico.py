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
