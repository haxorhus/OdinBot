import cv2
import numpy as np
import matplotlib.pyplot as plt

# Ruta de la imagen
image_path = "/home/jose/microros_ws/src/img/img.png"

# Longitud real del lado del área de trabajo (en metros)
real_side_length = 1.0

# --- Funciones auxiliares ---
def find_corners(points):
    """Encuentra las esquinas del área de trabajo basándose en los puntos detectados."""
    top_left = min(points, key=lambda p: (p[0] + p[1]))
    top_right = max(points, key=lambda p: (p[0] - p[1]))
    bottom_left = min(points, key=lambda p: (p[0] - p[1]))
    bottom_right = max(points, key=lambda p: (p[0] + p[1]))
    return top_left, top_right, bottom_left, bottom_right

def calculate_overlap(bbox1, bbox2):
    """Calcula el porcentaje de superposición entre dos bounding boxes."""
    x1, y1, x2, y2 = bbox1
    x1_p, y1_p, x2_p, y2_p = bbox2
    dx = min(x2, x2_p) - max(x1, x1_p)
    dy = min(y2, y2_p) - max(y1, y1_p)
    if dx >= 0 and dy >= 0:
        overlap_area = dx * dy
        bbox1_area = (x2 - x1) * (y2 - y1)
        return overlap_area / bbox1_area
    return 0

def detect_valid_objects(mask, label, color, robot_bbox=None, threshold=0.05):
    """Detecta objetos válidos según la máscara proporcionada."""
    positions = []
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        if cv2.contourArea(contour) > 200:
            x, y, w, h = cv2.boundingRect(contour)
            bbox = (x, y, x + w, y + h)
            center_x, center_y = x + w // 2, y + h // 2

            # Ignorar obstáculos superpuestos con el robot
            if robot_bbox and calculate_overlap(robot_bbox, bbox) >= threshold:
                continue

            positions.append((center_x * scale_factor, center_y * scale_factor))
            cv2.rectangle(warped_img, (x, y), (x + w, y + h), color, 2)
            cv2.putText(warped_img, label, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    return positions

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

top_left, top_right, bottom_left, bottom_right = find_corners(filtered_vertices)

# --- Transformación de perspectiva ---
output_width, output_height = 500, 500
source_points = np.array([top_left, top_right, bottom_left, bottom_right], dtype="float32")
destination_points = np.array([[0, 0], [output_width - 1, 0], [0, output_height - 1], [output_width - 1, output_height - 1]], dtype="float32")
matrix = cv2.getPerspectiveTransform(source_points, destination_points)
warped_img = cv2.warpPerspective(img, matrix, (output_width, output_height))

# --- Factor de escala ---
scale_factor = real_side_length / output_width

# --- Detectar objetos ---
hsv_warped = cv2.cvtColor(warped_img, cv2.COLOR_BGR2HSV)
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

# Detectar color rojo (obstáculos)
lower_red1, upper_red1 = np.array([0, 100, 100]), np.array([10, 255, 255])
lower_red2, upper_red2 = np.array([160, 100, 100]), np.array([180, 255, 255])
red_mask = cv2.morphologyEx(cv2.bitwise_or(cv2.inRange(hsv_warped, lower_red1, upper_red1), cv2.inRange(hsv_warped, lower_red2, upper_red2)), cv2.MORPH_CLOSE, kernel)

# Detectar color verde (objetivo)
lower_green, upper_green = np.array([40, 50, 50]), np.array([80, 255, 255])
green_mask = cv2.morphologyEx(cv2.inRange(hsv_warped, lower_green, upper_green), cv2.MORPH_CLOSE, kernel)

# Detectar color azul (robot)
lower_blue, upper_blue = np.array([100, 50, 50]), np.array([140, 255, 255])
blue_mask = cv2.morphologyEx(cv2.morphologyEx(cv2.inRange(hsv_warped, lower_blue, upper_blue), cv2.MORPH_CLOSE, kernel), cv2.MORPH_OPEN, kernel)

# Detectar robot
contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
robot_position = None
robot_bbox = None
if contours:
    all_points = np.vstack([cv2.boundingRect(c)[:2] for c in contours])
    x, y, w, h = cv2.boundingRect(all_points)
    center_x, center_y = x + w // 2, y + h // 2
    robot_position = (center_x * scale_factor, center_y * scale_factor)
    robot_bbox = (x, y, x + w, y + h)
    cv2.rectangle(warped_img, (x, y), (x + w, y + h), (0, 255, 255), 3)
    cv2.putText(warped_img, "Robot", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

# Detectar obstáculos y objetivo
obstacle_positions = detect_valid_objects(red_mask, "Obstacle", (0, 255, 0), robot_bbox)
goal_positions = detect_valid_objects(green_mask, "Goal", (255, 0, 0))

def transform_coordinates(positions, real_side_length):
    """Transforma las coordenadas al nuevo sistema de referencia."""
    transformed = []
    for x, y in positions:
        new_x = real_side_length - y
        new_y = real_side_length - x
        transformed.append((round(new_x, 2), round(new_y, 2)))
    return transformed

# Transformar coordenadas
transformed_obstacle_positions = transform_coordinates(obstacle_positions, real_side_length)
transformed_goal_positions = transform_coordinates(goal_positions, real_side_length)
transformed_robot_position = None
if robot_position:
    transformed_robot_position = transform_coordinates([robot_position], real_side_length)[0]

# Imprimir coordenadas transformadas
print("Coordenadas transformadas de obstáculos:", transformed_obstacle_positions)
print("Coordenadas transformadas del objetivo:", transformed_goal_positions)
if transformed_robot_position:
    print("Coordenadas transformadas del robot:", transformed_robot_position)
else:
    print("No se detectó el robot.")

# --- Mostrar resultados ---
plt.figure(figsize=(10, 5))  # Ajuste del tamaño de la figura

plt.subplot(1, 2, 1)
plt.title("Imagen Original")
plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
plt.axis("off")

plt.subplot(1, 2, 2)
plt.title("Resultados Finales")
plt.imshow(cv2.cvtColor(warped_img, cv2.COLOR_BGR2RGB))
plt.axis("off")

plt.tight_layout()
plt.show()
