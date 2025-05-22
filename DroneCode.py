import cv2
import numpy as np
from djitellopy import Tello
import time

# ---------------- Variables globales de control PI ----------------
integral_x_distance = 0
integral_angle_error = 0
last_seen = "center"  # Ayuda a corregir si se pierde la línea

# ---------------- Función principal de control ----------------
def follow_line(tello, x_pos, y_pos, bottom_linecx, bottom_linecy, top_linecx, top_linecy,
                kp=0.2, ki=0.001, kp_yaw=0.6, ki_yaw=0.001, integral_limit=50):
    global integral_x_distance, integral_angle_error, last_seen

    if bottom_linecx is not None:
        # Cálculo de errores
        x_distance = bottom_linecx - x_pos
        angle = -((-np.degrees(np.arctan2((top_linecy - bottom_linecy), top_linecx - bottom_linecx))) - 90)

        # Registro de la última dirección
        if x_distance > 20:
            last_seen = "right"
        elif x_distance < -20:
            last_seen = "left"
        else:
            last_seen = "center"

        # Control proporcional
        lateral_p = kp * x_distance
        yaw_p = kp_yaw * angle

        # Control integral con anti-windup
        integral_x_distance = np.clip(integral_x_distance + x_distance, -integral_limit, integral_limit)
        integral_angle_error = np.clip(integral_angle_error + angle, -integral_limit, integral_limit)

        lateral_i = ki * integral_x_distance
        yaw_i = ki_yaw * integral_angle_error

        # Señales de control finales
        lateral_vel = int(np.clip(lateral_p + lateral_i, -100, 100))
        vel_yaw = int(np.clip(yaw_p + yaw_i, -100, 100))

        tello.send_rc_control(lateral_vel, 15, 0, vel_yaw)

# ---------------- Modo de corrección cuando no hay línea ----------------
def recalibrate(tello):
    global last_seen
    if last_seen == "right":
        tello.send_rc_control(30, 5, 0, 0)
    elif last_seen == "left":
        tello.send_rc_control(-30, 5, 0, 0)
    else:
        tello.send_rc_control(0, 10, 0, 0)

# ---------------- Procesamiento de imagen ----------------
def process_img(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 70])
    mask = cv2.inRange(hsv, lower_black, upper_black)   
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    cleaned = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel, iterations=2)

    height = cleaned.shape[0]
    top_half = cleaned[0:height//2, :]
    bottom_half = cleaned[height//2:, :]

    def find_centroid(image_part, offset_y=0):
        contours, _ = cv2.findContours(image_part, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour) > 400:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"]) + offset_y
                    return cx, cy
        return None, None

    top_cx, top_cy = find_centroid(top_half)
    bottom_cx, bottom_cy = find_centroid(bottom_half, offset_y=height//2)

    cv2.imshow('Binary View', cleaned)
    return (top_cx, top_cy), (bottom_cx, bottom_cy)

# ---------------- Obtención del centro del frame ----------------
def get_frame_centroid(frame):
    height, width, _ = frame.shape
    return width // 2, height // 2

# ---------------- Bucle principal ----------------
def tracking_loop(tello):
    frame_reader = tello.get_frame_read()
    time.sleep(1)

    while True:
        frame = cv2.resize(frame_reader.frame, (480, 360))
        frame = cv2.flip(frame, 0)

        x_pos, y_pos = get_frame_centroid(frame)
        (top_cx, top_cy), (bottom_cx, bottom_cy) = process_img(frame)

        if bottom_cx is not None and top_cx is not None:
            cv2.line(frame, (bottom_cx, bottom_cy), (top_cx, top_cy), (0, 255, 0), 2)
            cv2.circle(frame, (top_cx, top_cy), 5, (255, 0, 0), -1)
            cv2.circle(frame, (bottom_cx, bottom_cy), 5, (0, 0, 255), -1)
            follow_line(tello, x_pos, y_pos, bottom_cx, bottom_cy, top_cx, top_cy)
        else:
            recalibrate(tello)

        cv2.imshow("Tello Line Tracking", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# ---------------- Inicialización del dron ----------------
def init():
    tello = Tello()
    tello.connect()
    print("Battery:", tello.get_battery(), "%")
    #tello.takeoff()
    tello.streamon()
    try:
        tracking_loop(tello)
    finally:
        tello.send_rc_control(0, 0, 0, 0)
        tello.land()
        print("Landing... Battery:", tello.get_battery(), "%")
        tello.streamoff()
        cv2.destroyAllWindows()

# ---------------- Punto de entrada ----------------
if __name__ == "__main__":
    init()