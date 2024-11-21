import cv2
import numpy as np
import os

# Crear carpeta para guardar las imágenes si no existe
output_folder = "../img/"
os.makedirs(output_folder, exist_ok=True)

# URL de la cámara web
#Enlace al servidor RTSP de la marca Dahua
PASS = "pass1234"#input("Ingrese la Contraseña Administador del dispositivo: ")
IP = "192.168.1.64"#4nput("Ingrese la direccion IP: ")
CH = "1"#input("Ingrese el numero del canal: ")
#URL = "rtsp://admin:"+PASS+"@"+IP+":554/cam/realmonitor?channel="+CH+"&subtype=0"
URL = "rtsp://admin:"+PASS+"@"+IP+":554/Streaming/Channels/"+CH+"01"
URL_Cam_IP = "rtsp://"+IP+":554/Streaming/Channels/"+CH+"01"
cap = cv2.VideoCapture(URL)

def is_rectangle(approx):
    """Verifica si una forma aproximada es un rectángulo."""
    return len(approx) == 4

def process_frame(frame):
    """
    Procesa un frame para:
    - Detectar contornos y formas.
    - Identificar rectángulos y cuadrados.
    - Crear una imagen binaria con rectángulos.
    """
    # Convertir a escala de grises
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Aplicar desenfoque para reducir ruido
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Detectar bordes usando Canny
    edges = cv2.Canny(blurred, 50, 150)

    # Encontrar contornos
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Imagen binaria para rectángulos
    rect_image = np.ones_like(gray) * 255  # Fondo blanco


    return rect_image, frame

def main():

    if not cap.isOpened():
        print("No se pudo abrir la cámara.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error al capturar el frame.")
            break

        # Guardar la imagen binaria si se presiona 's'
        if cv2.waitKey(1) & 0xFF == ord('s'):
            cv2.imwrite("rectangles_only.png", frame)
            print("Imagen binaria guardada como 'rectangles_only.png'.")

        # Salir si se presiona 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Liberar recursos
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
