"""Genera las texturas ArUco de los equipos a escala reglamentaria.

Uso básico:
    cd src/vsss_sim/models/robot/materials/textures
    python3 generar_arucos.py

Escribe los PNG de los seis robots en el directorio actual; las imágenes son
de 800 x 800 píxeles, con un marcador de 700 píxeles y márgenes de 50 píxeles.
"""

import cv2
import cv2.aruco as aruco
import numpy as np
import os

# Escala: 10 pixeles = 1 mm
tamano_imagen = 800  # 80 mm totales
margen_blanco = 50   # 5 mm de margen
tamano_aruco = 700   # 70 mm de ArUco

equipos = {
    'azul': {'ids': [256, 272, 273], 'color_fondo': (255, 0, 0), 'color_figura': (255, 255, 255)},
    'amarillo': {'ids': [955, 771, 939], 'color_fondo': (0, 0, 0), 'color_figura': (0, 255, 255)}
}

try:
    diccionario = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
except AttributeError:
    diccionario = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)

os.makedirs('.', exist_ok=True)

for equipo, datos in equipos.items():
    for id_robot in datos['ids']:
        try:
            img_aruco = aruco.generateImageMarker(diccionario, id_robot, tamano_aruco)
        except AttributeError:
            img_aruco = aruco.drawMarker(diccionario, id_robot, tamano_aruco)

        img_color = cv2.cvtColor(img_aruco, cv2.COLOR_GRAY2BGR)

        mask_negro = cv2.inRange(img_color, (0, 0, 0), (50, 50, 50))
        img_color[mask_negro > 0] = datos['color_fondo']
        mask_blanco = cv2.inRange(img_color, (200, 200, 200), (255, 255, 255))
        img_color[mask_blanco > 0] = datos['color_figura']

        lienzo = np.full((tamano_imagen, tamano_imagen, 3), 255, dtype=np.uint8)
        lienzo[margen_blanco:margen_blanco + tamano_aruco, margen_blanco:margen_blanco + tamano_aruco] = img_color

        nombre_archivo = f'aruco_{equipo}_{id_robot}.png'
        cv2.imwrite(nombre_archivo, lienzo)
