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
