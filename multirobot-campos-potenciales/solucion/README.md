# Implementación Física de un Sistema Multi-Robot Reactivo Basado en Campos Potenciales

El robot atacante deberá perseguir al robot protegido, mientras que el robot guardaespaldas deberá impedir dinámicamente dicha interacción mediante comportamientos reactivos autónomos. 


## Condiciones

El robot protegido debe realizar un movimiento libre, el robot atacante tiene como objetivo al protegido pero su obstáculo es el robot defensor. Este último tiene como meta el punto medio del vector entre el atacante y el protegido. 


## Flujo de solución (propuesta):

El movimiento del robot protegido es establecido de manera manual por un control de xbox mientras que el desplazamiento de los demás robots se resuelve con campos potenciales.

1. El código debe permitir la detección de los ArUcos y al mismo tiempo mover el robot protegido (ArUco + XboX.2. Integrar el código del atacante para seguir al protegido (Campos Potenciales + ArUco + XboX).
3. Verificar que al detectar un tercer ArUco, ninguno de los dos robots choque con él (ni atacante ni protegido). En otro caso realizar cambios necesarios.
4. Calcular la distancia entre el protegido y el atacante para dibujar un punto en la ventana.
5. Integrar la lógica del defensor para seguir el punto medio del vector entre el atacante y el protegido. 

## Comandos adicionales

v4l2-ctl --list-devices

source venv/bin/activate

 

