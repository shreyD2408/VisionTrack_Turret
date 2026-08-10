import cv2
import numpy as np
import serial
import time

Kp = 0.08
Ki = 0.0
Kd = 0.05

integral = 0
previous_error = 0
previous_time = time.time()

servo_angle = 90

SERIAL_PORT = "COM3"
BAUD_RATE = 9600
CAMERA_INDEX = 1

LOWER_YELLOW = np.array([25, 80, 60])
UPPER_YELLOW = np.array([40, 255, 255])

MIN_AREA = 100

MIN_ASPECT_RATIO = 0.2
MAX_ASPECT_RATIO = 5.0

SERIAL_INTERVAL = 0.01

SMOOTHING = 0.25

arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)

camera = cv2.VideoCapture(CAMERA_INDEX)
if not camera.isOpened():
    print("Could not open camera")
    arduino.close()
    exit()
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

smoothed_x = None
last_send_time = 0
last_detection_time = time.time()

previous_time = time.time()

while True:
    success, frame = camera.read()
    if not success:
        print("Failed to capture frame")
        break

    height, width = frame.shape[:2]
    center_x = width // 2
    center_y = height // 2

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best_contour = None
    best_area = 0

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < MIN_AREA:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        aspect_ratio = w / float(h)

        if aspect_ratio < MIN_ASPECT_RATIO:
            continue
        if aspect_ratio > MAX_ASPECT_RATIO:
            continue

        if area > best_area:
            best_area = area
            best_contour = contour

    if best_contour is not None:
        x, y, w, h = cv2.boundingRect(best_contour)
        object_x = x + w // 2

        error = object_x - center_x

        if abs(error) < 10:
            error = 0

        current_time = time.time()

        dt = current_time - previous_time
        if dt > 0:
            integral += error * dt
            derivative = (error - previous_error) / dt

            output = Kp * error + Ki * integral + Kd * derivative
            servo_angle += output

            servo_angle = np.clip(servo_angle, 0, 180)

            previous_error = error
            previous_time = current_time

            arduino.write(f"{int(servo_angle)}\n".encode())

        object_y = y + h // 2

        if smoothed_x is None:
            smoothed_x = object_x
        else:
            smoothed_x = (SMOOTHING * object_x + (1- SMOOTHING) * smoothed_x)
        smoothed_x = int(smoothed_x)

        current_time = time.time()
        if (current_time - last_send_time >= SERIAL_INTERVAL):
            last_send_time = current_time
        last_detection_time = current_time

        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(frame, (smoothed_x, object_y), 6, (0, 0, 255), -1)
        cv2.putText(frame, f"X: {smoothed_x}", (x, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame, f"Area: {int(best_area)}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    else:
        cv2.putText(frame, "TARGET NOT FOUND", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    cv2.line(frame, (center_x, 0), (center_x, height), (255, 0, 0), 2)
    cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)

    current_time = time.time()
    fps = 1 / (current_time - previous_time)
    previous_time = current_time
    cv2.putText(frame, f"FPS: {fps:.1f}", (20, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    cv2.imshow("Smooth Tracking", frame)
    cv2.imshow("Mask", mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
camera.release()
arduino.close()
cv2.destroyAllWindows()

