import cv2
import numpy as np
import matplotlib.pyplot as plt
import queue
import threading
import time
import os

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
video_url = "http://172.16.235.51:8080/video"  # URL de la cámara IP
cap = VideoCapture(video_url)

# --- Longitud real del lado del área de trabajo (en metros) ---
real_side_length = 2.7

# Variable global para la matriz de transformación guardada
saved_matrix = None

# Carpeta para guardar imágenes
output_folder = "/home/jose/microros_ws/src/img/cropped_image.png"


# Tiempo de la última imagen guardada
last_save_time = time.time()

# --- Función para encontrar esquinas del área de trabajo ---
def find_corners(points):
    """Encuentra las esquinas del área de trabajo basándose en los puntos detectados."""
    top_left = min(points, key=lambda p: (p[0] + p[1]))
    top_right = max(points, key=lambda p: (p[0] - p[1]))
    bottom_left = min(points, key=lambda p: (p[0] - p[1]))
    bottom_right = max(points, key=lambda p: (p[0] + p[1]))
    return top_left, top_right, bottom_left, bottom_right

# --- Procesamiento en tiempo real ---
while True:
    # Leer el fotograma actual
    frame = cap.read()
    #frame = frame[0:720, 120:1100]

    # Convertir a escala de grises y suavizar
    gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur_img = cv2.GaussianBlur(gray_img, (7, 7), 0)

    # Detectar bordes
    edges_img = cv2.Canny(blur_img, 100, 400)

    # Detectar líneas y esquinas
    lines = cv2.HoughLinesP(edges_img, 1, np.pi / 180, 80, minLineLength=100, maxLineGap=30)
    line_segments = [(line[0][0:2], line[0][2:4]) for line in lines] if lines is not None else []

    # Filtrar puntos válidos
    image_height, image_width = frame.shape[:2]
    filtered_vertices = [
        tuple(p) for segment in line_segments for p in segment
        if 0 <= p[0] < image_width and 0 <= p[1] < image_height
    ]
    filtered_vertices = list(set(filtered_vertices))  # Eliminar duplicados

    # Si se detectaron suficientes puntos, calcular la matriz de transformación
    if len(filtered_vertices) >= 4:
        top_left, top_right, bottom_left, bottom_right = find_corners(filtered_vertices)

        # Calcular tamaño de salida basado en la resolución deseada
        map_resolution = 0.005  # Resolución en metros/píxel
        real_side_length = 2.7  # Dimensión real del lado en metros

        output_size = int(real_side_length / map_resolution)  # Tamaño en píxeles
        output_width, output_height = output_size, output_size

        # Transformación de perspectiva
        source_points = np.array([top_left, top_right, bottom_left, bottom_right], dtype="float32")
        destination_points = np.array([
            [0, 0],
            [output_width - 1, 0],
            [0, output_height - 1],
            [output_width - 1, output_height - 1]
        ], dtype="float32")

        matrix = cv2.getPerspectiveTransform(source_points, destination_points)

        # Si hay una matriz guardada, usarla
        if saved_matrix is not None:
            warped_img = cv2.warpPerspective(frame, saved_matrix, (output_width, output_height))
        else:
            warped_img = cv2.warpPerspective(frame, matrix, (output_width, output_height))

        # Mostrar resultados
        cv2.imshow("Vista Cenital", warped_img)

        # Guardar la imagen transformada cada 0.3 segundos
        current_time = time.time()
        if current_time - last_save_time >= 0.3:
            save_path = output_folder
            cv2.imwrite(save_path, warped_img)
            last_save_time = current_time
            print(f"Imagen guardada en {save_path}")

    # Mostrar el video original
    cv2.imshow("Video Original", frame)

    # Manejar eventos de teclado
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('s'):  # Guardar la matriz actual
        if 'matrix' in locals():
            saved_matrix = matrix
            print("Matriz de transformación guardada y aplicada.")
        else:
            print("No hay una matriz de transformación para guardar.")

# Liberar recursos
cap.cap.release()
cv2.destroyAllWindows()
