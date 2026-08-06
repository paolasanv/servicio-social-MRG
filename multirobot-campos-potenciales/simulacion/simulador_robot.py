import pygame
import math

# ======================================================
# CONFIGURACIÓN
# ======================================================

ANCHO = 1200
ALTO = 700
FPS = 60

# Escala
PIXELS_POR_METRO = 100

# Robot
RADIO = 18

# Velocidades máximas
V_MAX = 0.8          # m/s
W_MAX = 2.0          # rad/s

# Colores
BLANCO = (255,255,255)
NEGRO = (0,0,0)
AZUL = (40,100,255)
ROJO = (255,60,60)
VERDE = (40,200,40)
GRIS = (180,180,180)

# ======================================================
# INICIALIZACIÓN
# ======================================================

pygame.init()

screen = pygame.display.set_mode((ANCHO, ALTO))
pygame.display.set_caption("Simulación Robot Diferencial")

clock = pygame.time.Clock()
font = pygame.font.SysFont("Arial",20)

# ======================================================
# ESTADO DEL ROBOT
# ======================================================

x = ANCHO/2
y = ALTO/2

theta = 0.0

trayectoria = []

objetivo = (1000,120)

# ======================================================
# BUCLE PRINCIPAL
# ======================================================

running = True

while running:

    dt = clock.tick(FPS)/1000.0

    # --------------------------------------------------
    # Eventos
    # --------------------------------------------------

    for event in pygame.event.get():

        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.KEYDOWN:

            if event.key == pygame.K_c:
                trayectoria.clear()

    # --------------------------------------------------
    # Lectura del teclado
    # --------------------------------------------------

    keys = pygame.key.get_pressed()

    v = 0.0
    w = 0.0

    if keys[pygame.K_UP]:
        v = V_MAX

    if keys[pygame.K_DOWN]:
        v = -V_MAX

    if keys[pygame.K_LEFT]:
        w = -W_MAX

    if keys[pygame.K_RIGHT]:
        w = W_MAX

    # --------------------------------------------------
    # Cinemática del robot diferencial
    # --------------------------------------------------

    theta += w*dt

    x += v*math.cos(theta)*dt*PIXELS_POR_METRO
    y += v*math.sin(theta)*dt*PIXELS_POR_METRO

    # Limitar a la ventana
    x = max(RADIO,min(ANCHO-RADIO,x))
    y = max(RADIO,min(ALTO-RADIO,y))

    trayectoria.append((x,y))

    # --------------------------------------------------
    # Dibujar
    # --------------------------------------------------

    screen.fill(BLANCO)

    # Cuadrícula

    for xx in range(0,ANCHO,100):
        pygame.draw.line(screen,GRIS,(xx,0),(xx,ALTO),1)

    for yy in range(0,ALTO,100):
        pygame.draw.line(screen,GRIS,(0,yy),(ANCHO,yy),1)

    # Trayectoria

    if len(trayectoria)>1:
        pygame.draw.lines(screen,(120,120,120),False,trayectoria,2)

    # Objetivo

    pygame.draw.circle(screen,ROJO,objetivo,8)

    pygame.draw.line(screen,ROJO,
                     (objetivo[0]-10,objetivo[1]),
                     (objetivo[0]+10,objetivo[1]),2)

    pygame.draw.line(screen,ROJO,
                     (objetivo[0],objetivo[1]-10),
                     (objetivo[0],objetivo[1]+10),2)

    # Robot

    pygame.draw.circle(screen,AZUL,(int(x),int(y)),RADIO)

    # Orientación

    punta_x = x + 35*math.cos(theta)
    punta_y = y + 35*math.sin(theta)

    pygame.draw.line(screen,
                     NEGRO,
                     (x,y),
                     (punta_x,punta_y),
                     4)

    pygame.draw.circle(screen,NEGRO,(int(punta_x),int(punta_y)),4)

    # --------------------------------------------------
    # Información
    # --------------------------------------------------

    xm = x/PIXELS_POR_METRO
    ym = y/PIXELS_POR_METRO

    distancia = math.sqrt((objetivo[0]-x)**2+(objetivo[1]-y)**2)/PIXELS_POR_METRO

    texto = [

        f"Posición X : {xm:.2f} m",
        f"Posición Y : {ym:.2f} m",
        f"Orientación : {math.degrees(theta)%360:.1f}°",
        f"Velocidad lineal : {v:.2f} m/s",
        f"Velocidad angular : {w:.2f} rad/s",
        f"Distancia al objetivo : {distancia:.2f} m",
        "",
        "Controles",
        "↑ Avanzar",
        "↓ Retroceder",
        "← Girar izquierda",
        "→ Girar derecha",
        "C Borrar trayectoria",
        "Cerrar ventana para salir"

    ]

    ytxt = 10

    for t in texto:
        img = font.render(t,True,NEGRO)
        screen.blit(img,(10,ytxt))
        ytxt += 24

    pygame.display.flip()

pygame.quit()