# robot_movil.py

# ======================================================
# IMPORTACIÓN DE LIBRERÍAS
# ======================================================

# Librería utilizada para establecer la comunicación UDP
# entre el programa de control y el robot.
import socket

# Librería utilizada para realizar operaciones matemáticas
# y trabajar con vectores, ángulos y funciones trigonométricas.
import numpy as np

# Importación de los parámetros de configuración del robot.
#
# UDP_PORT: puerto utilizado para la comunicación UDP.
# R: radio de las ruedas.
# L: distancia entre las ruedas.
from src.config import UDP_PORT, R, L


# ======================================================
# UTILIDADES
# ======================================================

def _wrap(angulo):
    """
    Normaliza un ángulo al intervalo [-pi, pi].

    Esto permite representar cualquier ángulo mediante un
    intervalo único y evita problemas cuando un ángulo pasa
    de pi a -pi o viceversa.

    Parámetros:
        angulo: ángulo en radianes.

    Retorna:
        Ángulo equivalente dentro del intervalo [-pi, pi].
    """

    # Se utiliza atan2 junto con seno y coseno para obtener
    # una representación equivalente del ángulo dentro del
    # intervalo [-pi, pi].
    return np.arctan2(np.sin(angulo), np.cos(angulo))


# ======================================================
# CLASE ROBOT
# ======================================================

class Robot:
    """
    Representa un robot móvil de accionamiento diferencial.

    La clase permite:
    - Configurar la comunicación con el robot mediante UDP.
    - Convertir velocidades lineales y angulares en velocidades
      de las ruedas.
    - Enviar las velocidades de las ruedas.
    - Detener el robot.
    - Aplicar diferentes estrategias de control mediante
      campos potenciales.
    """

    def __init__(self, ip, kv, kw):
        """
        Inicializa un robot.

        Parámetros:
            ip: dirección IP del robot.
            kv: ganancia asociada a la velocidad lineal.
            kw: ganancia asociada a la velocidad angular.
        """

        # Dirección IP del robot al que se enviarán los comandos.
        self.ip = ip

        # Ganancia utilizada para calcular la velocidad lineal.
        self.kv = kv

        # Ganancia utilizada para calcular la velocidad angular.
        self.kw = kw

        # Velocidad lineal máxima permitida.
        self.v_max = 3

        # Velocidad angular máxima permitida.
        self.w_max = 1

        # Distancia de seguridad utilizada alrededor de un robot
        # en los campos potenciales.
        self.dist_seguridad = 0.55

        # Distancia máxima a partir de la cual se considera la
        # influencia repulsiva de un obstáculo.
        self.dist_repulsion = 0.55

        # Puerto UDP utilizado para enviar las velocidades.
        self.port = UDP_PORT

        # Creación del socket UDP.
        self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

    # ======================================================
    # CINEMÁTICA DEL ROBOT
    # ======================================================

    def calcular_velocidades_ruedas(self, v, w):
        """
        Convierte la velocidad lineal y angular del robot
        en velocidades angulares para las ruedas.

        El robot utiliza una configuración diferencial.

        Parámetros:
            v: velocidad lineal del robot.
            w: velocidad angular del robot.

        Retorna:
            vr: velocidad angular de la rueda derecha.
            vl: velocidad angular de la rueda izquierda.
        """

        # Velocidad de la rueda derecha.
        vr = (v + w * L / 2) / R

        # Velocidad de la rueda izquierda.
        vl = (v - w * L / 2) / R

        return vr, vl

    # ======================================================
    # COMUNICACIÓN CON EL ROBOT
    # ======================================================

    def enviar_velocidades(self, vr, vl):
        """
        Envía las velocidades de las ruedas al robot mediante UDP.

        Parámetros:
            vr: velocidad angular de la rueda derecha.
            vl: velocidad angular de la rueda izquierda.
        """

        # Construcción del mensaje que será enviado.
        #
        # Las velocidades se representan con tres cifras decimales
        # y se separan mediante una coma.
        mensaje = f"{vr:.3f},{vl:.3f}"

        # Envío del mensaje mediante UDP.
        self.sock.sendto(mensaje.encode(),(self.ip, self.port))

        # Impresión de las velocidades enviadas para
        # facilitar la supervisión del sistema.
        print(
            f"Enviado -> derecha: {vr:+.3f} rad/s | "
            f"izquierda: {vl:+.3f} rad/s"
        )

    def detener(self):
        """
        Detiene el robot enviando velocidad cero a ambas ruedas.
        """

        self.enviar_velocidades(0, 0)

    def cerrar(self):
        """
        Cierra el socket utilizado para la comunicación UDP.
        """

        self.sock.close()

    # ======================================================
    # UTILIDADES COMUNES PARA CAMPOS POTENCIALES
    # ======================================================

    def _repulsion(self,xr,yr,xo,yo,d0,k_rep,gx,gy,k_tang):
        """
        Calcula la fuerza repulsiva producida por un obstáculo
        sobre el robot.

        El obstáculo se encuentra en (xo, yo) y el robot en
        (xr, yr).

        La fuerza resultante está formada por:

        - Una componente radial que aleja al robot del obstáculo.
        - Una componente tangencial que permite rodear el obstáculo.

        Parámetros:
            xr, yr:
                Posición del robot.

            xo, yo:
                Posición del obstáculo.

            d0:
                Distancia máxima de influencia del obstáculo.

            k_rep:
                Ganancia de la componente repulsiva.

            gx, gy:
                Vector unitario que apunta hacia la meta.

            k_tang:
                Peso de la componente tangencial.

        Retorna:
            fx, fy:
                Componentes x e y de la fuerza repulsiva.
        """

        # Vector que va desde el obstáculo hasta el robot.
        dx, dy = xr - xo, yr - yo

        # Distancia entre el robot y el obstáculo.
        d = np.hypot(dx, dy)

        # Si el robot está prácticamente sobre el obstáculo
        # o fuera de su radio de influencia, no se genera
        # fuerza repulsiva.
        if d < 1e-6 or d >= d0:
            return 0.0, 0.0

        # Vector unitario que apunta desde el obstáculo
        # hacia el robot.
        ux, uy = dx / d, dy / d

        # Magnitud de la componente radial de repulsión.
        #
        # La fuerza disminuye linealmente al acercarse al
        # límite d0.
        mag = k_rep * (d0 - d) / d0

        # Producto cruzado entre la dirección radial y
        # la dirección hacia la meta.
        #
        # Su signo determina hacia qué lado se debe generar
        # la componente tangencial.
        cruz = ux * gy - uy * gx

        # Selección del sentido de la componente tangencial.
        signo = 1.0 if cruz >= 0 else -1.0

        # Componente x de la fuerza resultante.
        fx = (mag * ux+ signo * k_tang * mag * (-uy))

        # Componente y de la fuerza resultante.
        fy = (mag * uy+ signo * k_tang * mag * (ux))

        return fx, fy

    def _fuerza_a_velocidades(self,fx,fy,theta_r,umbral=1e-3):
        """
        Convierte una fuerza resultante en una velocidad lineal
        y una velocidad angular para un robot diferencial.

        Parámetros:
            fx, fy:
                Componentes de la fuerza resultante.

            theta_r:
                Orientación actual del robot.

            umbral:
                Magnitud mínima de fuerza necesaria para producir
                movimiento.

        Retorna:
            v: velocidad lineal.
            w: velocidad angular.
        """

        # Magnitud de la fuerza resultante.
        magnitud = np.hypot(fx, fy)

        # Si la fuerza es demasiado pequeña, el robot permanece
        # detenido.
        if magnitud < umbral:
            return 0.0, 0.0

        # Dirección de la fuerza resultante.
        theta_resultante = np.arctan2(fy, fx)

        # Error angular entre la dirección deseada y la orientación
        # actual del robot.
        theta_e = _wrap(theta_resultante - theta_r)

        # Sentido de desplazamiento.
        sentido = 1.0

        # Si la fuerza apunta hacia la parte posterior del robot,
        # se modifica la dirección para permitir que el robot
        # retroceda en lugar de girar 180 grados.
        if abs(theta_e) > np.pi / 2:

            # Se cambia la dirección deseada 180 grados.
            theta_e = _wrap(theta_e + np.pi)

            # Indica que el movimiento debe realizarse
            # hacia atrás.
            sentido = -1.0

        # Cálculo de la velocidad lineal.
        #
        # La velocidad se reduce mediante cos(theta_e) cuando
        # el robot no está completamente alineado con la fuerza.
        v = (sentido* self.kv* magnitud* max(0.0, np.cos(theta_e)))

        # Cálculo de la velocidad angular.
        #
        # Se conserva la convención de signo definida
        # para el controlador.
        w = -self.kw * np.sin(theta_e)

        # Limitación de la velocidad lineal.
        v = np.clip(v,-self.v_max,self.v_max)

        # Limitación de la velocidad angular.
        w = np.clip(w,-self.w_max,self.w_max)

        return v, w

    # ======================================================
    # CAMPOS POTENCIALES
    # ======================================================

    def control_potencial(self,xr,yr,theta_r,xo,yo,k_r):
        """
        Control básico mediante un campo potencial atractivo.

        El robot se dirige hacia una posición objetivo.

        Parámetros:
            xr, yr:
                Posición actual del robot.

            theta_r:
                Orientación actual del robot.

            xo, yo:
                Posición del objetivo.

            k_r:
                Saturación máxima del error de distancia.

        Retorna:
            v: velocidad lineal.
            w: velocidad angular.
        """

        # Vector desde el robot hasta el objetivo.
        dx = xo - xr
        dy = yo - yr

        # Distancia entre el robot y el objetivo.
        d = np.hypot(dx, dy)

        # Orientación necesaria para dirigirse hacia el objetivo.
        theta_g = np.arctan2(dy, dx)

        # Error angular entre la orientación deseada y
        # la orientación actual.
        theta_e = np.arctan2(np.sin(theta_g - theta_r),np.cos(theta_g - theta_r))

        # Error de distancia considerando la distancia
        # de seguridad.
        error = max(0.0,d - self.dist_seguridad)

        # Velocidad lineal proporcional al error de distancia,
        # limitada por k_r.
        v = self.kv * min(error, k_r)

        # Velocidad angular para orientar el robot hacia
        # el objetivo.
        w = -self.kw * np.sin(theta_e)

        # Si el robot ya está dentro de la distancia de seguridad,
        # se detiene completamente.
        if d <= self.dist_seguridad:
            v = 0
            w = 0

        # Limitación de la velocidad lineal.
        v = np.clip(v,0,self.v_max)

        # Limitación de la velocidad angular.
        w = np.clip(w,-self.w_max,self.w_max)

        return v, w

    def control_potencial_atrac_rep(self,xr,yr,theta_r,xo,yo,k_r,k_rep):
        """
        Versión original del controlador que combina un campo
        atractivo y uno repulsivo.

        Esta versión no considera la presencia de un defensor
        y se conserva por compatibilidad con otras partes
        del programa.

        Parámetros:
            xr, yr:
                Posición del robot.

            theta_r:
                Orientación del robot.

            xo, yo:
                Posición del objetivo.

            k_r:
                Ganancia/saturación del campo atractivo.

            k_rep:
                Ganancia de la componente repulsiva.

        Retorna:
            v: velocidad lineal.
            w: velocidad angular.
        """

        # Vector desde el robot hacia el objetivo.
        dx = xo - xr
        dy = yo - yr

        # Distancia hasta el objetivo.
        d = np.hypot(dx, dy)

        # Dirección angular hacia el objetivo.
        theta_g = np.arctan2(dy, dx)

        # Error angular entre la dirección del objetivo
        # y la orientación del robot.
        theta_e = np.arctan2(np.sin(theta_g - theta_r),np.cos(theta_g - theta_r))

        # Error de distancia utilizado para el campo atractivo.
        error_atractivo = max(0.0,d - self.dist_seguridad)

        # Componente lineal atractiva.
        v_atractiva = self.kv * min(error_atractivo,k_r)

        # Componente angular atractiva.
        w_atractiva = -self.kw * np.sin(theta_e)

        # Si el robot se encuentra dentro de la distancia
        # de seguridad, se calcula una componente repulsiva.
        if d < self.dist_seguridad and d > 0:

            # Dirección desde el objetivo hacia el robot.
            theta_repulsiva = np.arctan2(yr - yo,xr - xo)

            # Error angular de la dirección repulsiva.
            theta_rep_error = np.arctan2(np.sin(theta_repulsiva - theta_r),np.cos(theta_repulsiva - theta_r))

            # Magnitud de la fuerza repulsiva.
            fuerza_repulsiva = (k_rep* (self.dist_seguridad - d))

            # Componente lineal repulsiva.
            v_repulsiva = fuerza_repulsiva

            # Componente angular repulsiva.
            w_repulsiva = (self.kw* np.sin(theta_rep_error))

        else:

            # Fuera de la distancia de seguridad no existe
            # componente repulsiva.
            v_repulsiva = 0.0
            w_repulsiva = 0.0

        # Selección del comportamiento según la distancia.
        if d >= self.dist_seguridad:

            # Cuando el robot está suficientemente alejado,
            # se utiliza el campo atractivo.
            v = v_atractiva
            w = w_atractiva

        else:

            # Cuando el robot está demasiado cerca,
            # se utiliza el comportamiento repulsivo.
            v = -v_repulsiva
            w = w_repulsiva

        # Limitación de la velocidad lineal.
        v = np.clip(v,-self.v_max,self.v_max)

        # Limitación de la velocidad angular.
        w = np.clip(w,-self.w_max,self.w_max)

        return v, w

    # ======================================================
    # ATACANTE:
    # META = PROTEGIDO
    # OBSTÁCULO = DEFENSOR
    # ======================================================

    def control_potencial_atacante(self,xr,yr,theta_r,protegido_x,protegido_y,defensor_x,defensor_y,k_r,k_rep,k_rep_obs,k_tang=1.0):
        """
        Control mediante campos potenciales para el atacante.

        El atacante tiene como objetivo acercarse al protegido,
        mientras considera al defensor como un obstáculo que
        debe evitar.

        Parámetros:
            xr, yr:
                Posición actual del atacante.

            theta_r:
                Orientación actual del atacante.

            protegido_x, protegido_y:
                Posición del robot protegido.

            defensor_x, defensor_y:
                Posición del defensor.

            k_r:
                Saturación del campo atractivo.

            k_rep:
                Ganancia de repulsión respecto al protegido
                cuando el atacante invade su distancia de seguridad.

            k_rep_obs:
                Ganancia de repulsión respecto al defensor.

            k_tang:
                Peso de la componente tangencial utilizada
                para rodear al defensor.

        Retorna:
            v: velocidad lineal.
            w: velocidad angular.
        """

        # ==================================================
        # 1) CAMPO ATRACTIVO HACIA EL PROTEGIDO
        # ==================================================

        # Vector desde el atacante hacia el protegido.
        dx = protegido_x - xr
        dy = protegido_y - yr

        # Distancia entre atacante y protegido.
        d = np.hypot(dx, dy)

        # Si ambos están prácticamente en la misma posición,
        # no se genera movimiento.
        if d < 1e-6:
            return 0.0, 0.0

        # Vector unitario que apunta hacia el protegido.
        gx, gy = dx / d, dy / d

        # Si el atacante está fuera de la distancia de seguridad,
        # se genera una fuerza atractiva hacia el protegido.
        if d >= self.dist_seguridad:

            # La fuerza se limita mediante k_r.
            f_att = min(d - self.dist_seguridad,k_r)

        else:
            # Si el atacante está demasiado cerca del protegido,
            # el campo cambia de atractivo a repulsivo.
            f_att = -k_rep * (self.dist_seguridad - d)

        # Componentes x e y de la fuerza atractiva.
        fx = f_att * gx
        fy = f_att * gy

        # ==================================================
        # 2) REPULSIÓN DEL DEFENSOR
        # ==================================================

        # Cálculo de la fuerza repulsiva generada por
        # el defensor considerado como obstáculo.
        f_ox, f_oy = self._repulsion(xr,yr,defensor_x,defensor_y,self.dist_repulsion,k_rep_obs,gx,gy,k_tang)

        # Suma de la fuerza repulsiva a la fuerza atractiva.
        fx += f_ox
        fy += f_oy

        # ==================================================
        # 3) CAMPO RESULTANTE -> VELOCIDADES
        # ==================================================

        # Conversión de la fuerza resultante en velocidad
        # lineal y angular.
        return self._fuerza_a_velocidades(fx,fy,theta_r)

    # ======================================================
    # DEFENSOR:
    # META = PUNTO MEDIO
    # OBSTÁCULOS = PROTEGIDO Y ATACANTE
    # ======================================================

    def control_potencial_defensor(self,xr,yr,theta_r,objetivo_x,objetivo_y,protegido_x,protegido_y,atacante_x,atacante_y,kr,krep,k_tang=0.6,tol_meta=0.05):
        """
        Control mediante campos potenciales para el defensor.

        El defensor se dirige hacia un punto objetivo, normalmente
        situado entre el atacante y el protegido, mientras utiliza
        ambos robots como obstáculos que debe evitar.

        Parámetros:
            xr, yr:
                Posición actual del defensor.

            theta_r:
                Orientación actual del defensor.

            objetivo_x, objetivo_y:
                Coordenadas del punto objetivo.

            protegido_x, protegido_y:
                Posición del protegido, considerado obstáculo.

            atacante_x, atacante_y:
                Posición del atacante, considerado obstáculo.

            kr:
                Ganancia del campo atractivo.

            krep:
                Ganancia de los campos repulsivos.

            k_tang:
                Peso de la componente tangencial.

            tol_meta:
                Tolerancia para considerar que se alcanzó la meta.

        Retorna:
            v: velocidad lineal.
            w: velocidad angular.
        """

        # ==================================================
        # 1) CAMPO ATRACTIVO HACIA EL PUNTO MEDIO
        # ==================================================

        # Vector desde el defensor hasta el punto objetivo.
        dx_obj = objetivo_x - xr
        dy_obj = objetivo_y - yr

        # Distancia desde el defensor hasta el objetivo.
        d_obj = np.hypot(dx_obj,dy_obj)

        # Si existe una distancia significativa hasta el objetivo,
        # se calcula el vector unitario hacia él.
        if d_obj > 1e-6:
            gx, gy = (dx_obj / d_obj,dy_obj / d_obj)

        else:

            # Si prácticamente no existe distancia, se establece
            # el vector dirección en cero.
            gx, gy = 0.0, 0.0

        # Magnitud del campo atractivo.
        #
        # Se limita la distancia utilizada en el cálculo a 1.0.
        magnitud_atractiva = kr * min(d_obj,1.0)

        # Componentes x e y del campo atractivo.
        f_ax = magnitud_atractiva * gx
        f_ay = magnitud_atractiva * gy

        # ==================================================
        # 2) RADIO DE REPULSIÓN DINÁMICO
        # ==================================================

        # Distancia entre el atacante y el protegido.
        d_ap = np.hypot(atacante_x - protegido_x,atacante_y - protegido_y)

        # El radio de repulsión se ajusta dinámicamente.
        #
        # Se utiliza el menor valor entre:
        # - la distancia de repulsión definida para el robot;
        # - el 40 % de la distancia entre atacante y protegido.
        #
        # Esto evita que los radios de repulsión sean demasiado
        # grandes cuando ambos robots se encuentran cerca.
        d0 = min(self.dist_repulsion,0.4 * d_ap)

        # --------------------------------------------------
        # Repulsión generada por el protegido
        # --------------------------------------------------

        f_px, f_py = self._repulsion(xr,yr,protegido_x,protegido_y,d0,krep,gx,gy,k_tang)

        # --------------------------------------------------
        # Repulsión generada por el atacante
        # --------------------------------------------------

        f_qx, f_qy = self._repulsion(xr,yr,atacante_x,atacante_y,d0,krep,gx,gy,k_tang)

        # ==================================================
        # 3) CAMPO RESULTANTE
        # ==================================================

        # Suma del campo atractivo y de las dos componentes
        # repulsivas.
        fx = f_ax + f_px + f_qx
        fy = f_ay + f_py + f_qy

        # ==================================================
        # ZONA MUERTA EN LA META
        # ==================================================

        # Si el defensor se encuentra suficientemente cerca
        # del objetivo y la fuerza resultante es pequeña,
        # se considera que ha alcanzado la meta.
        if (d_obj < tol_meta and np.hypot(fx, fy) < 0.05):
            return 0.0, 0.0

        # Conversión del campo resultante a velocidad lineal
        # y velocidad angular.
        return self._fuerza_a_velocidades(fx,fy,theta_r)
