# vision.py

import cv2 as cv
import cv2.aruco as aruco
import numpy as np
from collections import defaultdict


class Vision:

    def __init__(self):

        # Cámara
        self.cap = None
        self.window_name = "Campos Potenciales + ArUco"

        # ArUco
        self.MARKER_SIZE_METERS = 0.15
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(
            self.aruco_dict,
            self.parameters
        )

        # Parámetros cámara
        camera_params = [ 1999.17838, 2006.06097, 928.811574, 478.347147]

        self.camera_matrix = np.array(
            [
                [camera_params[0], 0, camera_params[2]],
                [0, camera_params[1], camera_params[3]],
                [0, 0, 1]
            ],
            dtype=np.float32
        )

        self.dist_coeffs = np.array( [0.0906, 0.3189, -0.0028, 0.00018, -2.998])

        # Filtros
        self.alpha = 0.3
        self.tvec_filt = defaultdict(lambda: None)
        self.yaw_filt = defaultdict(lambda: None)

    # ==================================================
    # CÁMARA
    # ==================================================

    def iniciar_camara(self):

        self.cap = cv.VideoCapture(2, cv.CAP_V4L2)

        if not self.cap.isOpened():
            raise RuntimeError("No se pudo abrir la cámara.")

        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 1080)

        self.cap.set(cv.CAP_PROP_AUTOFOCUS, 0)
        self.cap.set(cv.CAP_PROP_FOCUS, 0)

        cv.namedWindow(self.window_name, cv.WINDOW_NORMAL)
        cv.resizeWindow(self.window_name, 1920, 1080)

        print("Cámara inicializada correctamente.")

    # ==================================================
    # TRANSFORMACIONES
    # ==================================================

    def obtener_matriz_homogenea(self, rvec, tvec):
        R, _ = cv.Rodrigues(rvec)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = tvec
        return T

    def transformar_a_referencia(self, T_obj, T_ref):
        return np.linalg.inv(T_ref) @ T_obj

    # ==================================================
    # OBTENER POSES
    # ==================================================

    def obtener_poses(self):

        if self.cap is None:
            raise RuntimeError("Primero llama a iniciar_camara().")

        ret, frame = self.cap.read()

        if not ret:
            return None, {}, -1

        poses = {}

        corners, ids, _ = self.detector.detectMarkers(frame)

        if ids is not None:

            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.MARKER_SIZE_METERS, self.camera_matrix,self.dist_coeffs)
            poses_camara = {}

            for i, mid in enumerate(ids.flatten()):
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]

                # -------------------------
                # Filtro posición
                # -------------------------

                if self.tvec_filt[mid] is None:
                    self.tvec_filt[mid] = tvec
                else:
                    self.tvec_filt[mid] = (
                        self.alpha * tvec
                        + (1 - self.alpha) * self.tvec_filt[mid])

                # -------------------------
                # Orientación
                # -------------------------

                R, _ = cv.Rodrigues(rvec)
                yaw = np.arctan2(R[1, 0], R[0, 0])

                if self.yaw_filt[mid] is None:
                    self.yaw_filt[mid] = yaw
                else:
                    self.yaw_filt[mid] = np.arctan2(
                        self.alpha*np.sin(yaw)
                        + (1-self.alpha)*np.sin(self.yaw_filt[mid]),
                        self.alpha*np.cos(yaw)
                        + (1-self.alpha)*np.cos(self.yaw_filt[mid])
                    )

                poses_camara[mid] = (rvec,self.tvec_filt[mid])

                # -------------------------
                # Dibujos
                # -------------------------

                c = corners[i][0].astype(int)
                cv.polylines(frame,[c],True,(0, 255, 0),2)
                cx, cy = np.mean(c, axis=0).astype(int)
                cv.putText(frame, f"ID {mid}", (cx-20, cy-10), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, self.tvec_filt[mid], 0.05)

            # -------------------------
            # Transformar respecto al ArUco 0
            # -------------------------

            if 0 in poses_camara:

                T0 = self.obtener_matriz_homogenea(*poses_camara[0])

                for mid in poses_camara:
                    if mid == 0:
                        continue

                    T = self.obtener_matriz_homogenea(*poses_camara[mid])
                    T = self.transformar_a_referencia(T,T0)

                    poses[mid] = {
                        "x": float(T[0, 3]),
                        "y": float(T[1, 3]),
                        "theta": np.arctan2(T[1, 0], T[0, 0])  # self.yaw_filt[mid]
                    }

                    c = corners[np.where(ids.flatten() == mid)[0][0]][0].astype(int)
                    cx, cy = np.mean(c, axis=0).astype(int)
                    cv.circle(frame, (cx, cy), 70, (0, 0, 255), 2)
       
        # -------------------------
        # Vector atacante -> protegido
        # -------------------------

        self.dibujar_vector_atacante(frame,corners,ids)
        cv.imshow(self.window_name, frame)
        tecla = cv.waitKey(1) & 0xFF
        return frame, poses, tecla
    
    def dibujar_vector_atacante(self, frame, corners, ids):
        """
        Dibuja una flecha desde el ArUco 2 (atacante)
        hacia el ArUco 1 (protegido).
        """

        if ids is None:
            return

        ids_lista = ids.flatten()

        # Necesitamos que estén presentes ambos robots
        if 1 not in ids_lista or 2 not in ids_lista:
            return

        # Índices de los ArUcos
        idx_atacante = np.where(ids_lista == 2)[0][0]
        idx_protegido = np.where(ids_lista == 1)[0][0]

        # Esquinas
        esquinas_atacante = corners[idx_atacante][0]
        esquinas_protegido = corners[idx_protegido][0]

        # Centro de cada ArUco
        centro_atacante = np.mean(esquinas_atacante,axis=0 ).astype(int)
        centro_protegido = np.mean(esquinas_protegido,axis=0).astype(int)

        # Dibujar flecha 2 -> 1
        cv.arrowedLine(
            frame,
            tuple(centro_atacante),
            tuple(centro_protegido),
            (255, 0, 255),   # magenta
            5,               # grosor
            tipLength=0.08)

    # ==================================================
    # CIERRE
    # ==================================================

    def cerrar(self):
        if self.cap is not None:
            self.cap.release()
        cv.destroyAllWindows()
        cv.waitKey(1)
