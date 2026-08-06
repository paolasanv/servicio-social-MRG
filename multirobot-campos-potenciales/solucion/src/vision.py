# vision.py

import cv2 as cv
import cv2.aruco as aruco
import numpy as np

from collections import defaultdict


class Vision:

    def __init__(self):

        # ===========================
        # CONFIGURACIÓN ARUCO
        # ===========================

        self.MARKER_SIZE_METERS = 0.1
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)

        # ===========================
        # CÁMARA
        # ===========================

        self.cap = cv.VideoCapture(2, cv.CAP_V4L2)

        if not self.cap.isOpened():
            raise RuntimeError("No se pudo abrir la cámara.")

        # Resolución
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 1080)

        # Enfoque manual
        self.cap.set(cv.CAP_PROP_AUTOFOCUS, 0)
        FOCUS_3M = 0
        self.cap.set(cv.CAP_PROP_FOCUS, FOCUS_3M)

        # ===========================
        # PARÁMETROS DE LA CÁMARA
        # ===========================

        camera_params = [1999.17838, 2006.06097, 928.811574, 478.347147]
        self.camera_matrix = np.array(
            [[camera_params[0], 0, camera_params[2]],
             [0, camera_params[1], camera_params[3]],
             [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.array([0.0906, 0.3189, -0.0028, 0.00018, -2.998])

        # ===========================
        # FILTROS EMA
        # ===========================
        self.alpha = 0.3
        self.tvec_filt = defaultdict(lambda: None)
        self.yaw_filt = defaultdict(lambda: None)

        # ===========================
        # VENTANA
        # ===========================
        cv.namedWindow("Campos Potenciales + ArUco", cv.WINDOW_NORMAL)
        cv.resizeWindow(self.window_name, 1920, 1080)

    # =============================
    # TRANSFORMACIONES
    # =============================

    def obtener_matriz_homogenea(self, rvec, tvec):
        R, _ = cv.Rodrigues(rvec)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = tvec
        return T

    def transformar_a_referencia(self, T_obj, T_ref):
        return np.linalg.inv(T_ref) @ T_obj


    # ==============================
    # OBTENER POSES DE LOS ARUCOS
    # ==============================

    def obtener_poses(self):
        ret, frame = self.cap.read()

        if not ret:
            return None, {}

        poses = {}
        corners, ids, _ = self.detector.detectMarkers(frame)

        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.MARKER_SIZE_METERS, self.camera_matrix, self.dist_coeffs)

            poses_camara = {}

            for i, mid in enumerate(ids.flatten()):
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]

                # =========================
                # FILTRO EMA DE POSICIÓN
                # =========================

                if self.tvec_filt[mid] is None:
                    self.tvec_filt[mid] = tvec
                else:
                    self.tvec_filt[mid] = (self.alpha * tvec + (1 - self.alpha) * self.tvec_filt[mid])

                # =========================
                # ORIENTACIÓN
                # =========================

                R, _ = cv.Rodrigues(rvec)
                yaw = np.arctan2(R[1, 0], R[0, 0])

                if self.yaw_filt[mid] is None:
                    self.yaw_filt[mid] = yaw

                else:
                    self.yaw_filt[mid] = np.arctan2(
                        self.alpha * np.sin(yaw) + (1 - self.alpha) * np.sin(self.yaw_filt[mid]),
                        self.alpha * np.cos(yaw) + (1 - self.alpha) * np.cos(self.yaw_filt[mid]))

                poses_camara[mid] = (rvec, self.tvec_filt[mid])

                # ==========================
                # DIBUJO
                # ==========================

                c = corners[i][0].astype(int)
                cv.polylines(frame, [c], True, (0, 255, 0), 2)

                cx, cy = np.mean(c, axis=0).astype(int)
                cv.putText(frame, f"ID {mid}", (cx - 20, cy - 10), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                cv.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, self.tvec_filt[mid], 0.05)

            # =============================
            # TRANSFORMAR AL SISTEMA DEL ARUCO 0
            # =============================

            if 0 in poses_camara:

                T0 = self.obtener_matriz_homogenea(*poses_camara[0])

                for mid in poses_camara:

                    if mid == 0:
                        continue

                    T = self.obtener_matriz_homogenea(*poses_camara[mid])

                    T = self.transformar_a_referencia(T, T0)

                    poses[mid] = {
                        "x": float(T[0, 3]),
                        "y": float(T[1, 3]),
                        "theta": self.yaw_filt[mid]
                    }

                    # círculo únicamente para visualizar

                    if mid != 0: #el unico sin circulo debe ser el aruco 0
                        c = corners[np.where(ids.flatten() == mid)[0][0]][0].astype(int)
                        cx, cy = np.mean(c, axis=0 ).astype(int)
                        cv.circle(frame, (cx, cy), 70, (0, 0, 255), 2)

        cv.imshow(self.window_name, frame)
        return frame, poses

    # =========
    # CIERRE
    # =========

    def cerrar(self):
        self.cap.release()
        cv.destroyAllWindows()
        cv.waitKey(1)