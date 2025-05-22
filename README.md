# Dron Seguidor de Línea con Tello y Control PI

Este proyecto fue desarrollado como parte de la materia Implementación de Sistemas Mecatrónicos. El objetivo es convertir un dron DJI Tello en un seguidor de línea autónomo, utilizando visión por computadora y control PI para mantener la trayectoria.

## ¿Qué es un DJI Tello?

El DJI Tello es un minidron programable con cámara integrada, ideal para educación en robótica y visión artificial. A través de la librería `djitellopy`, es posible conectarse al dron, recibir imágenes en tiempo real y enviar comandos de movimiento.

## Objetivo del Proyecto

Desarrollar un sistema autónomo en el que el dron:
- Detecte una línea negra sobre el suelo usando visión por computadora.
- Siga esta línea de forma autónoma.
- Se mantenga centrado usando un control PI sintonizado experimentalmente.

## Componentes del sistema

- Dron: DJI Tello
- Librerías: `cv2`, `numpy`, `djitellopy`
- Cámara: Integrada en el Tello (stream de video)
- PC: Ejecuta el script Python para visión y control

## Funcionamiento

1. Adquisición de imagen: Se procesa el video del Tello en tiempo real.
2. Detección de línea: Se convierte la imagen a HSV y se binariza para detectar una línea negra.
3. Cálculo de centroides: Se identifican los centros de la línea en dos zonas (superior e inferior) para estimar el ángulo.
4. Control PI:
   - El error lateral (distancia del centro de la imagen al centroide de la línea) y el ángulo se utilizan como entradas.
   - Se aplica control Proporcional-Integral (PI) para generar las velocidades laterales (left/right) y de giro (yaw).
5. Modo recuperación: Si no se detecta línea, se aplica una estrategia correctiva según la última dirección conocida.

## Sintonización del Control PI

Durante las pruebas, se ajustaron las constantes `Kp` y `Ki` tanto para el desplazamiento lateral como para el giro:

- `kp`, `ki`: control lateral
- `kp_yaw`, `ki_yaw`: control de orientación

Se probaron diferentes métodos para encontrar los parámetros adecuados:
- Ajuste manual empírico
- Observación de oscilaciones
- Limitación de la integral para evitar "windup"

## Estructura del Código

- `process_img(frame)`: Procesa el frame y devuelve los centroides
- `follow_line(...)`: Aplica el control PI para seguir la línea
- `recalibrate(...)`: Recuperación cuando no se detecta la línea
- `tracking_loop(tello)`: Ciclo principal de seguimiento
- `init()`: Inicializa el Tello, establece la conexión y ejecuta el ciclo

## Posibles mejoras

- Agregar control derivativo (PID) para mayor precisión
- Implementar un filtro adaptativo HSV con trackbars
- Grabar los vuelos y errores para analizar el rendimiento
- Agregar seguimiento de curva suave con anticipación
- Entrenar una red neuronal ligera para detección más robusta

## Autor

Proyecto desarrollado por Kleber Molina como parte del curso de Implementación de Sistemas Mecatrónicos en el Tecnológico de Monterrey.
