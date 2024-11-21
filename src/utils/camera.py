import cv2
import numpy as np

# Dirección de la cámara IP (reemplaza esto con tu URL de cámara IP)
video_url = "http://192.168.240.229:8080/video"

# Inicializar el stream de video
cap = cv2.VideoCapture(2)
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
    cv2.drawContours(frame, contours, -1, (255, 0, 0), 2)  # Contornos en azul

    # Convertir la imagen al espacio HSV para detección de colores
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # --- DETECCIÓN DE OBJETOS ROJOS ---
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)

    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)

    mask_red = mask_red1 + mask_red2

    # Encontrar contornos de objetos rojos
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(frame, contours_red, -1, (255, 0, 0), 2)  # Contornos en rojo

    # --- DETECCIÓN DE OBJETOS VERDES ---
    lower_green = np.array([40, 40, 40])
    upper_green = np.array([80, 255, 255])
    mask_green = cv2.inRange(hsv, lower_green, upper_green)

    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(frame, contours_green, -1, (0, 0, 255), 2)  # Contornos en verde

    # --- DETECCIÓN DEL CÍRCULO (ROBOT) ---
    gray_blurred = cv2.medianBlur(gray, 5)
    circles = cv2.HoughCircles(
        gray_blurred,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=30,
        param1=150,
        param2=40,
        minRadius=10,
        maxRadius=30
    )

    
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for circle in circles[0, :]:
            cx, cy, radius = circle
            cv2.circle(frame, (cx, cy), radius, (255, 255, 0), 2)  # Contorno del círculo en cian
            cv2.circle(frame, (cx, cy), 2, (0, 255, 255), 3)  # Centro del círculo en amarillo
    

    # Mostrar el video con los contornos
    cv2.imshow('Video con Contornos', frame)

    # Presionar 'q' para salir del bucle
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.imwrite("../img/img.png", frame)
        break

# Liberar recursos
cap.release()
cv2.destroyAllWindows()
