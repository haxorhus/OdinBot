import cv2
import numpy as np
import os 
from matplotlib import pyplot as plt
import math
import time

#CONSTANTES DE INTERES
proporcion_x = 0.7535 
proporcion_y = 0.4557
altura_camara = 0.27 #metros este parametro hay que verificar al momento de implementar
W_m = 2 * altura_camara * proporcion_x #metros
H_m = 2 * altura_camara * proporcion_y #metros
lado_robot = 0.025 #metros
W = 1280 #Corresponde a X cm
H = 720 #Corresponde a Y cm
num_div_x = int(W_m/lado_robot) # este numero debe ser igual a la proporcion entre la altura del frame y la altura del robot
num_div_y = int(H_m/lado_robot) # este numero debe ser igual a la proporcion entre la anchura del frame y la anchura del robot
nw = int(W/num_div_x) #pixeles => lado_robot [m]
nh = int(H/num_div_y) #pixeles => lado_robot [m]

last_time_render = 0

#MASCARA PARA DETECCION DE CUERPOS EN MOVIMINETO
detection = cv2.createBackgroundSubtractorMOG2(history=10000,varThreshold=12)

#limites de la meta
lower_green = np.array([40, 100, 100],np.uint8)# meta => verde
upper_green = np.array([100, 255, 255],np.uint8)
#Limites de la Caja
lower_yellow = np.array([20, 100, 100],np.uint8)#Caja => yellow
upper_yellow = np.array([30, 255, 255],np.uint8)
#Limite del robot
blueBajo = np.array([100,100,50],np.uint8)#robot => azul[100,100,50][140,255,100]
blueAlto = np.array([140,255,100],np.uint8)
#Limite de los obstaculos
obstBajo = np.array([160,150,0],np.uint8) # obstaculos => rojo
obstAlto = np.array([180,255,255],np.uint8)

#Enlace al servidor RTSP de la marca Dahua
PASS = "pass1234"#input("Ingrese la Contraseña Administador del dispositivo: ")
IP = "192.168.1.64"#input("Ingrese la direccion IP: ")
CH = "1"#input("Ingrese el numero del canal: ")
#URL = "rtsp://admin:"+PASS+"@"+IP+":554/cam/realmonitor?channel="+CH+"&subtype=0"
URL = "rtsp://admin:"+PASS+"@"+IP+":554/Streaming/Channels/"+CH+"01"
URL_Cam_IP = "rtsp://"+IP+":554/Streaming/Channels/"+CH+"01"
Capture = cv2.VideoCapture(URL)

def ajustar_brillo(img, brillo_rojo=0, brillo_verde=0, brillo_azul = 0, constraste=0):
    """
    Ajusta el brillo de la imagen.
    
    Parámetros:
    - img: La imagen original (en formato BGR).
    - brillo: El nivel de ajuste de brillo, un valor entre -100 a 100.
    
    Retorna:
    - La imagen con el brillo ajustado.
    """
    b, g, r = cv2.split(img)
    #cv2.imshow("b",b)
    b_brillo= b + brillo_azul
    g_brillo= g + brillo_verde
    r_brillo= r + brillo_rojo
    r_brillo= np.clip(r_brillo, 0, 255)
    img = cv2.merge((b_brillo.astype(np.uint8), g_brillo.astype(np.uint8), r_brillo.astype(np.uint8)))
    img = img * constraste
    img = np.clip(img, 0, 255)
    return img.astype(np.uint8)
# Ajuste del brillo inicial (modifica este valor según sea necesario)
nivel_rojo = 0
nivel_verde = 5
nivel_azul = 0
nivel_constraste = 1.1


while True:
    #try:
        #Recuperamos los frames del video 1 a 1 en cada iteracion
        ret,frame = Capture.read()
        #frame = cv2.blur(frame, (5, 5))
        if ret == False:
            print("no se encontro frame")
        #print(frame.shape)

        #if time.time() - last_time_render >= 1:
        #last_time_render = time.time()
        # Factor de escala
        #scale_factor = 0.9  # Redimensiona al 50% del tamaño original

        # Obtener el nuevo tamaño
        #new_width = int(frame.shape[1] * scale_factor)
        #new_height = int(frame.shape[0] * scale_factor)

        # Redimensionar la imagen
        #frame = cv2.resize(frame, (new_width, new_height))

        #frame = ajustar_brillo(frame,nivel_rojo,nivel_verde,nivel_azul,nivel_constraste)
        cv2.imwrite("corvi.png", frame)


        cv2.imshow("Imagen para generar trayectoria",frame)

        t = cv2.waitKey(1)
        if t & 0xFF == ord('q'):
          break


Capture.release()
cv2.destroyAllWindows()