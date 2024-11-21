import cv2
import numpy as np

# Dirección de la cámara IP (reemplaza esto con tu URL de cámara IP)
video_url = "http://192.168.0.4:8080/video"

# Inicializar el stream de video
cap = cv2.VideoCapture(video_url)

# Variables de configuración
map_resolution = 0.005  # Resolución en metros/píxel
map_size_meters = 2.7   # Tamaño del mapa en metros


if not cap.isOpened():
    print("No se pudo abrir la cámara IP.")
    exit()

while True:
    # Capturar frame del stream
    ret, frame = cap.read()
    if not ret:
        print("No se pudo leer el frame del stream.")
        break

    # Convertir el frame a escala de grises
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Aplicar detección de bordes
    edges = cv2.Canny(gray, 50, 150)

    # Encontrar contornos
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Dibujar todos los contornos detectados
    cv2.drawContours(frame, contours, -1, (255, 0, 0), 2)  # Contornos en azu

    # Convertir la imagen a escala de grises
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Aplicar detección de bordes
    edges = cv2.Canny(gray, 50, 150)
    # Encontrar contornos
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filtrar contornos para encontrar el cuadrado más grande
    largest_contour = max(contours, key=cv2.contourArea)

    # Obtener el rectángulo delimitador del cuadrado
    x, y, w, h = cv2.boundingRect(largest_contour)

    # Ajusta w y h para que representen un cuadrado de 2.7x2.7 metros en píxeles
    desired_size_in_pixels = int(map_size_meters / map_resolution)
    cropped_image = cv2.resize(frame[y:y+h, x:x+w], (desired_size_in_pixels, desired_size_in_pixels))

    # Mostrar el video con los contornos
    cv2.imshow('Video cortado', cropped_image)

    # Presionar 'q' para salir del bucle
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.imwrite("../img/img.png", frame)
        break

# Liberar recursos
cap.release()
cv2.destroyAllWindows()
