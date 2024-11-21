import cv2
import numpy as np
import matplotlib.pyplot as plt

# Ruta de la imagen
image_path = "/home/jose/microros_ws/src/img/img.jpeg"

# Longitud real del lado del área de trabajo (en metros)
real_side_length = 2.7

# --- Función para encontrar esquinas del área de trabajo ---
def find_corners(points):
    """Encuentra las esquinas del área de trabajo basándose en los puntos detectados."""
    top_left = min(points, key=lambda p: (p[0] + p[1]))
    top_right = max(points, key=lambda p: (p[0] - p[1]))
    bottom_left = min(points, key=lambda p: (p[0] - p[1]))
    bottom_right = max(points, key=lambda p: (p[0] + p[1]))
    return top_left, top_right, bottom_left, bottom_right

# --- Cargar y procesar la imagen ---
img = cv2.imread(image_path)
gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
blur_img = cv2.GaussianBlur(gray_img, (7, 7), 0)
edges_img = cv2.Canny(blur_img, 100, 400)

# --- Detectar líneas y esquinas ---
lines = cv2.HoughLinesP(edges_img, 1, np.pi / 180, 80, minLineLength=100, maxLineGap=30)
line_segments = [(line[0][0:2], line[0][2:4]) for line in lines] if lines is not None else []

# Filtrar puntos válidos
image_height, image_width = img.shape[:2]
filtered_vertices = [
    tuple(p) for segment in line_segments for p in segment
    if 0 <= p[0] < image_width and 0 <= p[1] < image_height
]
filtered_vertices = list(set(filtered_vertices))  # Eliminar duplicados

# Encontrar esquinas del área de trabajo
if len(filtered_vertices) >= 4:
    top_left, top_right, bottom_left, bottom_right = find_corners(filtered_vertices)
else:
    raise ValueError("No se encontraron suficientes puntos para definir las esquinas.")

# --- Transformación de perspectiva ---
output_width, output_height = 500, 500
source_points = np.array([top_left, top_right, bottom_left, bottom_right], dtype="float32")
destination_points = np.array([[0, 0], [output_width - 1, 0], [0, output_height - 1], [output_width - 1, output_height - 1]], dtype="float32")
matrix = cv2.getPerspectiveTransform(source_points, destination_points)
warped_img = cv2.warpPerspective(img, matrix, (output_width, output_height))

# --- Mostrar resultados ---
plt.figure(figsize=(10, 5))  # Ajuste del tamaño de la figura

plt.subplot(1, 2, 1)
plt.title("Imagen Original")
plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
plt.axis("off")

plt.subplot(1, 2, 2)
plt.title("Vista Cenital")
plt.imshow(cv2.cvtColor(warped_img, cv2.COLOR_BGR2RGB))
cv2.imwrite("/home/jose/microros_ws/src/img/cropped_image.png", warped_img)
plt.axis("off")

plt.tight_layout()
plt.show()
