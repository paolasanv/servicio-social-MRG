# vision.py

# ==================================================
# IMPORTACIÓN DE LIBRERÍAS
# ==================================================

import cv2 as cv
import cv2.aruco as aruco
import numpy as np
from collections import defaultdict


class Vision:
    """
    Clase encargada de gestionar la cámara y realizar la detección
    y estimación de pose de marcadores ArUco.

    También realiza:
    - Filtrado de las posiciones y orientaciones detectadas.
    - Transformación de las poses respecto al ArUco de referencia (ID 0).
    - Visualización de los marcadores detectados.
    - Representación gráfica del vector entre el atacante (ID 2)
      y el protegido (ID 1).
    """

    def __init__(self):

        # ==================================================
        # CÁMARA
        # ==================================================

        # Objeto encargado de controlar la captura de imágenes
        # de la cámara. Inicialmente no se encuentra conectado.
        self.cap = None

        # Nombre de la ventana donde se mostrará la imagen
        # obtenida de la cámara.
        self.window_name = "Campos Potenciales + ArUco"

        # ==================================================
        # CONFIGURACIÓN DE ARUCO
        # ==================================================

        # Tamaño físico de cada marcador ArUco, expresado en metros.
        # Este valor es necesario para poder estimar la posición
        # tridimensional del marcador respecto a la cámara.
        self.MARKER_SIZE_METERS = 0.15

        # Selección del diccionario de marcadores ArUco.
        # En este caso se utiliza el diccionario 5x5 con 100
        # identificadores disponibles.
        self.aruco_dict = aruco.getPredefinedDictionary(
            aruco.DICT_5X5_100
        )

        # Parámetros utilizados durante la detección de los
        # marcadores ArUco.
        self.parameters = aruco.DetectorParameters()

        # Detector encargado de localizar los marcadores en
        # cada imagen obtenida por la cámara.
        self.detector = aruco.ArucoDetector(
            self.aruco_dict,
            self.parameters
        )

        # ==================================================
        # PARÁMETROS DE LA CÁMARA
        # ==================================================

        # Parámetros intrínsecos de la cámara:
        # [fx, fy, cx, cy]
        #
        # fx: distancia focal en el eje x.
        # fy: distancia focal en el eje y.
        # cx: coordenada x del centro óptico.
        # cy: coordenada y del centro óptico.
        camera_params = [
            1999.17838,
            2006.06097,
            928.811574,
            478.347147
        ]

        # Construcción de la matriz intrínseca de la cámara.
        #
        # Esta matriz permite relacionar las coordenadas 3D
        # de un objeto con su proyección sobre la imagen.
        self.camera_matrix = np.array(
            [
                [camera_params[0], 0, camera_params[2]],
                [0, camera_params[1], camera_params[3]],
                [0, 0, 1]
            ],
            dtype=np.float32
        )

        # Coeficientes de distorsión de la cámara.
        #
        # Estos valores permiten corregir las deformaciones
        # producidas por la lente de la cámara.
        self.dist_coeffs = np.array(
            [
                0.0906,
                0.3189,
                -0.0028,
                0.00018,
                -2.998
            ]
        )

        # ==================================================
        # FILTROS
        # ==================================================

        # Factor de suavizado utilizado por el filtro.
        #
        # Un valor de alpha cercano a 1 da mayor importancia
        # a la medición actual, mientras que un valor menor
        # produce mayor suavizado.
        self.alpha = 0.3

        # Diccionario utilizado para almacenar las posiciones
        # filtradas de cada marcador ArUco.
        #
        # Cada ID de ArUco tendrá asociada su última posición
        # filtrada.
        self.tvec_filt = defaultdict(lambda: None)

        # Diccionario utilizado para almacenar el ángulo yaw
        # filtrado de cada marcador ArUco.
        self.yaw_filt = defaultdict(lambda: None)

    # ==================================================
    # CÁMARA
    # ==================================================

    def iniciar_camara(self):
        """
        Inicializa la cámara y configura sus parámetros de captura.

        Se utiliza el dispositivo de cámara número 2 mediante
        la interfaz V4L2 de Linux.
        """

        # Creación del objeto de captura de video.
        self.cap = cv.VideoCapture(2, cv.CAP_V4L2)

        # Verificación de que la cámara se haya abierto
        # correctamente.
        if not self.cap.isOpened():
            raise RuntimeError("No se pudo abrir la cámara.")

        # Configuración de la resolución de captura.
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 1080)

        # Desactivación del enfoque automático.
        self.cap.set(cv.CAP_PROP_AUTOFOCUS, 0)

        # Establecimiento del enfoque en un valor fijo.
        self.cap.set(cv.CAP_PROP_FOCUS, 0)

        # Creación de la ventana donde se mostrará la imagen.
        cv.namedWindow(self.window_name, cv.WINDOW_NORMAL)

        # Establecimiento del tamaño de la ventana.
        cv.resizeWindow(self.window_name, 1920, 1080)

        print("Cámara inicializada correctamente.")

    # ==================================================
    # TRANSFORMACIONES
    # ==================================================

    def obtener_matriz_homogenea(self, rvec, tvec):
        """
        Construye una matriz de transformación homogénea 4x4
        a partir de un vector de rotación y un vector de traslación.

        Parámetros:
            rvec: vector de rotación obtenido mediante ArUco.
            tvec: vector de traslación obtenido mediante ArUco.

        Retorna:
            T: matriz homogénea que representa la posición y
               orientación del marcador.
        """

        # Conversión del vector de rotación a una matriz de
        # rotación 3x3 mediante la fórmula de Rodrigues.
        R, _ = cv.Rodrigues(rvec)

        # Inicialización de una matriz identidad 4x4.
        T = np.eye(4)

        # Se coloca la matriz de rotación en la parte superior
        # izquierda de la matriz homogénea.
        T[:3, :3] = R

        # Se coloca el vector de traslación en la última columna.
        T[:3, 3] = tvec

        return T

    def transformar_a_referencia(self, T_obj, T_ref):
        """
        Expresa la transformación de un objeto respecto a un
        sistema de referencia determinado.

        Parámetros:
            T_obj: transformación del objeto respecto a la cámara.
            T_ref: transformación del sistema de referencia
                   respecto a la cámara.

        Retorna:
            Transformación del objeto respecto al sistema de referencia.
        """

        # Se calcula la transformación relativa mediante:
        #
        # T_ref^-1 * T_obj
        #
        # De esta forma, las coordenadas del objeto quedan
        # expresadas respecto al sistema de coordenadas de referencia.
        return np.linalg.inv(T_ref) @ T_obj

    # ==================================================
    # OBTENER POSES
    # ==================================================

    def obtener_poses(self):
        """
        Captura una imagen, detecta los marcadores ArUco presentes
        y calcula sus posiciones y orientaciones respecto al ArUco 0.

        Retorna:
            frame: imagen capturada y procesada.
            poses: diccionario con las poses de los marcadores.
            tecla: tecla presionada por el usuario.
        """

        # Verificación de que la cámara haya sido inicializada.
        if self.cap is None:
            raise RuntimeError("Primero llama a iniciar_camara().")

        # Captura de un nuevo frame.
        ret, frame = self.cap.read()

        # Si no fue posible obtener la imagen, se retorna
        # una respuesta vacía.
        if not ret:
            return None, {}, -1

        # Diccionario donde se almacenarán las poses finales.
        poses = {}

        # Detección de los marcadores ArUco en la imagen.
        #
        # corners contiene las esquinas de cada marcador.
        # ids contiene los identificadores detectados.
        corners, ids, _ = self.detector.detectMarkers(frame)

        # Se continúa únicamente si se detectó al menos
        # un marcador.
        if ids is not None:

            # Estimación de la posición y orientación de cada
            # marcador respecto a la cámara.
            #
            # rvecs: vectores de rotación.
            # tvecs: vectores de traslación.
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners,
                self.MARKER_SIZE_METERS,
                self.camera_matrix,
                self.dist_coeffs
            )

            # Diccionario donde se almacenan temporalmente
            # las poses de los marcadores respecto a la cámara.
            poses_camara = {}

            # Recorrido de todos los marcadores detectados.
            for i, mid in enumerate(ids.flatten()):

                # Obtención de la rotación y traslación
                # correspondientes al marcador actual.
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]

                # -------------------------
                # FILTRO DE POSICIÓN
                # -------------------------

                # Si es la primera medición del marcador,
                # se utiliza directamente la posición obtenida.
                if self.tvec_filt[mid] is None:
                    self.tvec_filt[mid] = tvec

                else:
                    # Filtro exponencial para suavizar la posición.
                    #
                    # Se combina la posición actual con la
                    # posición filtrada anterior.
                    self.tvec_filt[mid] = (
                        self.alpha * tvec
                        + (1 - self.alpha) * self.tvec_filt[mid]
                    )

                # -------------------------
                # ORIENTACIÓN
                # -------------------------

                # Conversión del vector de rotación a una
                # matriz de rotación.
                R, _ = cv.Rodrigues(rvec)

                # Obtención del ángulo yaw a partir de la
                # matriz de rotación.
                #
                # atan2 permite obtener el ángulo teniendo
                # en cuenta el cuadrante correspondiente.
                yaw = np.arctan2(R[1, 0], R[0, 0])

                # Si es la primera medición, se utiliza
                # directamente el yaw obtenido.
                if self.yaw_filt[mid] is None:
                    self.yaw_filt[mid] = yaw

                else:
                    # Filtrado del ángulo utilizando seno y coseno.
                    #
                    # Esta representación evita problemas asociados
                    # al salto entre +pi y -pi.
                    self.yaw_filt[mid] = np.arctan2(
                        self.alpha * np.sin(yaw)
                        + (1 - self.alpha) * np.sin(
                            self.yaw_filt[mid]
                        ),
                        self.alpha * np.cos(yaw)
                        + (1 - self.alpha) * np.cos(
                            self.yaw_filt[mid]
                        )
                    )

                # Almacenamiento de la pose del marcador respecto
                # a la cámara.
                poses_camara[mid] = (
                    rvec,
                    self.tvec_filt[mid]
                )

                # -------------------------
                # DIBUJOS
                # -------------------------

                # Conversión de las esquinas del marcador
                # a coordenadas enteras de imagen.
                c = corners[i][0].astype(int)

                # Dibujo del contorno del marcador.
                cv.polylines(
                    frame,
                    [c],
                    True,
                    (0, 255, 0),
                    2
                )

                # Cálculo del centro del marcador.
                cx, cy = np.mean(c, axis=0).astype(int)

                # Escritura del identificador del marcador
                # sobre la imagen.
                cv.putText(
                    frame,
                    f"ID {mid}",
                    (cx - 20, cy - 10),
                    cv.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 0, 255),
                    2
                )

                # Dibujo de los ejes X, Y y Z del sistema de
                # coordenadas del marcador.
                cv.drawFrameAxes(
                    frame,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvec,
                    self.tvec_filt[mid],
                    0.05
                )

            # -------------------------
            # TRANSFORMAR RESPECTO AL ARUCO 0
            # -------------------------

            # El ArUco con ID 0 se utiliza como sistema de
            # referencia para los demás marcadores.
            if 0 in poses_camara:

                # Obtención de la matriz homogénea del ArUco 0.
                T0 = self.obtener_matriz_homogenea(
                    *poses_camara[0]
                )

                # Recorrido de los marcadores detectados.
                for mid in poses_camara:

                    # El marcador 0 ya es el sistema de referencia,
                    # por lo que no es necesario transformarlo.
                    if mid == 0:
                        continue

                    # Obtención de la matriz homogénea del marcador.
                    T = self.obtener_matriz_homogenea(
                        *poses_camara[mid]
                    )

                    # Transformación de la pose para expresarla
                    # respecto al ArUco 0.
                    T = self.transformar_a_referencia(T, T0)

                    # Almacenamiento de la posición y orientación
                    # relativa del marcador.
                    poses[mid] = {
                        "x": float(T[0, 3]),
                        "y": float(T[1, 3]),

                        # Obtención del ángulo theta a partir de
                        # la matriz de rotación relativa.
                        "theta": np.arctan2(
                            T[1, 0],
                            T[0, 0]
                        )
                    }

                    # Localización de las esquinas correspondientes
                    # al marcador actual.
                    c = corners[
                        np.where(ids.flatten() == mid)[0][0]
                    ][0].astype(int)

                    # Cálculo del centro del marcador.
                    cx, cy = np.mean(c, axis=0).astype(int)

                    # Dibujo de un círculo alrededor del marcador.
                    cv.circle(
                        frame,
                        (cx, cy),
                        70,
                        (0, 0, 255),
                        2
                    )

        # ==================================================
        # VECTOR ATACANTE -> PROTEGIDO
        # ==================================================

        # Dibuja una flecha desde el ArUco 2, correspondiente
        # al atacante, hacia el ArUco 1, correspondiente
        # al protegido.
        self.dibujar_vector_atacante(
            frame,
            corners,
            ids
        )

        # Mostrar el frame procesado en la ventana.
        cv.imshow(
            self.window_name,
            frame
        )

        # Lectura de una tecla presionada.
        tecla = cv.waitKey(1) & 0xFF

        # Retorno de la imagen, las poses calculadas
        # y la tecla presionada.
        return frame, poses, tecla

    def dibujar_vector_atacante(self, frame, corners, ids):
        """
        Dibuja una flecha desde el ArUco 2 (atacante)
        hacia el ArUco 1 (protegido).

        La flecha se utiliza como representación visual
        de la dirección atacante -> protegido.
        """

        # Si no existen marcadores detectados, no se
        # puede dibujar el vector.
        if ids is None:
            return

        # Conversión de los identificadores a una lista
        # unidimensional.
        ids_lista = ids.flatten()

        # Para poder dibujar el vector deben estar presentes
        # tanto el atacante (ID 2) como el protegido (ID 1).
        if 1 not in ids_lista or 2 not in ids_lista:
            return

        # ==================================================
        # ÍNDICES DE LOS MARCADORES
        # ==================================================

        # Obtención del índice correspondiente al ArUco 2.
        idx_atacante = np.where(
            ids_lista == 2
        )[0][0]

        # Obtención del índice correspondiente al ArUco 1.
        idx_protegido = np.where(
            ids_lista == 1
        )[0][0]

        # ==================================================
        # ESQUINAS DE LOS MARCADORES
        # ==================================================

        # Obtención de las esquinas del marcador atacante.
        esquinas_atacante = corners[idx_atacante][0]

        # Obtención de las esquinas del marcador protegido.
        esquinas_protegido = corners[idx_protegido][0]

        # ==================================================
        # CENTROS DE LOS MARCADORES
        # ==================================================

        # Cálculo del centro del marcador atacante.
        centro_atacante = np.mean(
            esquinas_atacante,
            axis=0
        ).astype(int)

        # Cálculo del centro del marcador protegido.
        centro_protegido = np.mean(
            esquinas_protegido,
            axis=0
        ).astype(int)

        # ==================================================
        # DIBUJAR FLECHA 2 -> 1
        # ==================================================

        # Dibujo de una flecha que representa la dirección
        # desde el atacante hacia el protegido.
        cv.arrowedLine(
            frame,
            tuple(centro_atacante),
            tuple(centro_protegido),
            (255, 0, 255),   # color magenta
            5,               # grosor de la línea
            tipLength=0.08
        )

    # ==================================================
    # CIERRE
    # ==================================================

    def cerrar(self):
        """
        Libera los recursos utilizados por la cámara y
        cierra todas las ventanas de OpenCV.
        """

        # Verificación de que la cámara haya sido inicializada.
        if self.cap is not None:

            # Liberación del dispositivo de captura.
            self.cap.release()

        # Cierre de todas las ventanas creadas por OpenCV.
        cv.destroyAllWindows()

        # Espera breve para garantizar que las ventanas
        # sean cerradas correctamente.
        cv.waitKey(1)
