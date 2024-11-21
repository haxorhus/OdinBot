import cv2
import numpy as np

# Cargar la imagen
image = cv2.imread('/home/jose/microros_ws/src/img/img.png')

# Convertir la imagen a escala de grises
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Aplicar detección de bordes
edges = cv2.Canny(gray, 50, 150)

# Encontrar contornos
contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Filtrar contornos para encontrar el cuadrado más grande
largest_contour = max(contours, key=cv2.contourArea)

# Obtener el rectángulo delimitador del cuadrado
x, y, w, h = cv2.boundingRect(largest_contour)

# Recortar la imagen
cropped_image = image[y:y+h, x:x+w]

# Convertir la imagen recortada al espacio HSV
hsv = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2HSV)

# --- DETECCIÓN DE OBJETOS ROJOS ---
# Rango de color rojo en HSV
lower_red = np.array([0, 120, 70])
upper_red = np.array([10, 255, 255])
mask_red1 = cv2.inRange(hsv, lower_red, upper_red)

lower_red2 = np.array([170, 120, 70])
upper_red2 = np.array([180, 255, 255])
mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)

# Unir las dos máscaras
mask_red = mask_red1 + mask_red2

# Crear imagen binaria (rojo como negro, fondo blanco)
binary_red = cv2.bitwise_not(mask_red)  # Invertir para fondo blanco
cv2.imshow("Objetos Rojos en Blanco y Negro", binary_red)
cv2.imwrite("../img/map.png",binary_red)

# --- DETECCIÓN DEL OBJETO VERDE ---
# Rango de color verde en HSV
lower_green = np.array([40, 40, 40])
upper_green = np.array([80, 255, 255])
mask_green = cv2.inRange(hsv, lower_green, upper_green)

# Encontrar contornos para el objeto verde
contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# --- DETECCIÓN DEL CÍRCULO (ROBOT) ---
gray_cropped = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)
gray_cropped = cv2.medianBlur(gray_cropped, 5)  # Reducir ruido

# Detección de círculos
circles = cv2.HoughCircles(
    gray_cropped, 
    cv2.HOUGH_GRADIENT, 
    dp=1.2, 
    minDist=30, 
    param1=150, 
    param2=40, 
    minRadius=10, 
    maxRadius=30
)

# Dimensiones reales del área en metros
real_width = 2.7  # metros
real_height = 2.7  # metros

# Dimensiones de la imagen en píxeles
image_height, image_width = cropped_image.shape[:2]

# Escala metros por píxel
scale_x = real_width / image_width
scale_y = real_height / image_height

# Coordenadas del objeto verde
if contours_green:
    largest_green = max(contours_green, key=cv2.contourArea)
    M_green = cv2.moments(largest_green)
    if M_green["m00"] != 0:
        cx_green_pixel = int(M_green["m10"] / M_green["m00"])
        cy_green_pixel = int(M_green["m01"] / M_green["m00"])
        
        # Convertir a coordenadas reales (metros)
        cx_green_real = scale_x * (image_width - cx_green_pixel)
        cy_green_real = scale_y * cy_green_pixel
        
        print(f"Centroide del objeto verde (en metros): ({cx_green_real:.2f}, {cy_green_real:.2f})")

# Coordenadas del círculo (robot)
if circles is not None:
    for circle in circles[0, :]:
        cx_circle_pixel, cy_circle_pixel, radius = circle
        
        # Convertir a coordenadas reales (metros)
        cx_circle_real = scale_x * (image_width - cx_circle_pixel)
        cy_circle_real = scale_y * cy_circle_pixel
        
        print(f"Centroide del círculo (robot) en metros: ({cx_circle_real:.2f}, {cy_circle_real:.2f})")


# Mostrar resultados
cv2.imshow('Cropped Image', cropped_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
