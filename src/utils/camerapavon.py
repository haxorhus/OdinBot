import cv2
import numpy as np
import queue
import threading
import time

# --- Clase para captura de video sin buffer ---
class VideoCapture:
    def __init__(self, name):
        self.cap = cv2.VideoCapture(name)
        self.q = queue.Queue()
        t = threading.Thread(target=self._reader)
        t.daemon = True
        t.start()

    def _reader(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            if not self.q.empty():
                try:
                    self.q.get_nowait()  # Descarta el fotograma anterior
                except queue.Empty:
                    pass
            self.q.put(frame)

    def read(self):
        return self.q.get()

# --- Configuración de video ---
video_url = "http://172.16.234.78:8080/video"  # URL de la cámara IP
cap = VideoCapture(video_url)

# Variables para almacenar las esquinas seleccionadas
corners = []
selecting = True

def select_corners(event, x, y, flags, param):
    global corners, selecting
    if event == cv2.EVENT_LBUTTONDOWN:
        corners.append((x, y))
        print(f"Esquina seleccionada: {x, y}")
        if len(corners) == 4:
            selecting = False

# Crear ventana para selección de esquinas
cv2.namedWindow("Seleccionar esquinas")
cv2.setMouseCallback("Seleccionar esquinas", select_corners)

print("Selecciona las cuatro esquinas del área de trabajo en el video.")

# Esperar hasta que se seleccionen las esquinas
while selecting:
    frame = cap.read()
    for corner in corners:
        cv2.circle(frame, corner, 5, (0, 0, 255), -1)
    cv2.imshow("Seleccionar esquinas", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("Selección cancelada.")
        selecting = False

cv2.destroyWindow("Seleccionar esquinas")

if len(corners) == 4:
    print("Esquinas seleccionadas:", corners)

    # Ordenar esquinas seleccionadas en el orden: top-left, top-right, bottom-left, bottom-right
    corners = np.array(corners, dtype="float32")
    s = corners.sum(axis=1)
    diff = np.diff(corners, axis=1)
    top_left = corners[np.argmin(s)]
    bottom_right = corners[np.argmax(s)]
    top_right = corners[np.argmin(diff)]
    bottom_left = corners[np.argmax(diff)]
    sorted_corners = np.array([top_left, top_right, bottom_left, bottom_right], dtype="float32")

    # Configurar la dimensión de la salida con resolución de 0.005 m/píxel y dimensiones reales de 3x2.4 m
    resolution = 0.005  # metros/píxel
    real_width = 3.0  # metros
    real_height = 2.4  # metros

    output_width = int(real_width / resolution)  # píxeles
    output_height = int(real_height / resolution)  # píxeles

    destination_points = np.array([
        [0, 0],
        [output_width - 1, 0],
        [0, output_height - 1],
        [output_width - 1, output_height - 1]
    ], dtype="float32")

    # Calcular la matriz de transformación
    matrix = cv2.getPerspectiveTransform(sorted_corners, destination_points)

    while True:
        frame = cap.read()
        cropped_img = cv2.warpPerspective(frame, matrix, (output_width, output_height))

        # Mostrar la imagen recortada
        cv2.imshow("Área de Trabajo Recortada", cropped_img)

        # Manejar eventos de teclado
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            save_path = "/home/jose/microros_ws/src/img/cropped_image.png"
            cv2.imwrite(save_path, cropped_img)
            print(f"Imagen guardada en {save_path}")

cv2.destroyAllWindows()
cap.cap.release()
