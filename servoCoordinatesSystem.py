import cv2
import numpy as np
import serial
import time

SERIAL_PORT = "COM3"
BAUD_RATE = 9600
CAMERA_INDEX = 1

LOWER_COLOR = np.array([15, 80, 50])
UPPER_COLOR = np.array([45, 255, 255])

MIN_AREA = 100
MIN_ASPECT_RATIO = 0.2
MAX_ASPECT_RATIO = 5.0

Kp = 0.08
Ki = 0.0
Kd = 0.01

MIN_SERVO_ANGLE = 20
MAX_SERVO_ANGLE = 160

servo_angle = 90

DEAD_ZONE = 5
SMOOTHING = 0.05

SERVO_UPDATE_INTERVAL = 0.05
MAX_SERVO_STEP = 10.0

integral = 0
previous_error = 0
previous_pid_time = time.time()

smoothed_x = None
last_servo_update = 0
previous_fps_time = time.time()

arduino = serial.Serial(
    SERIAL_PORT,
    BAUD_RATE,
    timeout=1

)

time.sleep(2)

camera = cv2.VideoCapture(CAMERA_INDEX)

if not camera.isOpened():
    print("Could not open camera")
    arduino.close()
    exit()

camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while True:

    success, frame = camera.read()

    if not success:
        print("Failed to capture camera")
        break

    height, width = frame.shape[:2]

    center_x = width // 2
    center_y = height // 2

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(
        hsv,
        LOWER_COLOR,
        UPPER_COLOR
    )

    kernel = np.ones(
        (5, 5),
        np.uint8
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        kernel
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        kernel
    )

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

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

        x, y, w, h = cv2.boundingRect(
            best_contour
        )

        object_x = x + w // 2
        object_y = y + h // 2

        if smoothed_x is None:
            smoothed_x = float(object_x)
        else:
            smoothed_x = (
                SMOOTHING * object_x
                + (1 - SMOOTHING) * smoothed_x
            )

        smoothed_x = float(smoothed_x)

        target_angle = np.interp(
            smoothed_x,
            [0, width],
            [
                MIN_SERVO_ANGLE,
                MAX_SERVO_ANGLE
            ]
        )

        error = target_angle - servo_angle

        if abs(error) < DEAD_ZONE:
            error = 0

        current_time = time.time()

        dt = current_time - previous_pid_time

        if dt > 0:

            integral += error * dt

            integral = np.clip(
                integral,
                -100,
                100
            )

            derivative = (
                error - previous_error
            ) / dt

            output = (
                Kp * error
                + Ki * integral
                + Kd * derivative
            )

            output = np.clip(
                output,
                -MAX_SERVO_STEP,
                MAX_SERVO_STEP
            )

            servo_angle += output

            servo_angle = np.clip(
                servo_angle,
                MIN_SERVO_ANGLE,
                MAX_SERVO_ANGLE
            )

            previous_error = error
            previous_pid_time = current_time

            if (
                current_time - last_servo_update
                >= SERVO_UPDATE_INTERVAL
            ):

                arduino.write(
                    f"{int(servo_angle)}\n".encode()
                )

                last_servo_update = current_time

        cv2.rectangle(
            frame,
            (x, y),
            (x + w, y + h),
            (0, 255, 0),
            2
        )

        cv2.circle(
            frame,
            (int(smoothed_x), object_y),
            6,
            (0, 0, 255),
            -1
        )

        cv2.putText(
            frame,
            f"X: {int(smoothed_x)}",
            (x, y - 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2
        )

        cv2.putText(
            frame,
            f"Target: {int(target_angle)}",
            (x, y - 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2
        )

        cv2.putText(
            frame,
            f"Servo: {int(servo_angle)}",
            (x, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame,
            f"Error: {error:.1f}",
            (20, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )

    else:

        cv2.putText(
            frame,
            "TARGET NOT FOUND",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2
        )

    cv2.line(
        frame,
        (center_x, 0),
        (center_x, height),
        (255, 0, 0),
        2
    )

    cv2.circle(
        frame,
        (center_x, center_y),
        5,
        (255, 0, 0),
        -1
    )

    current_fps_time = time.time()

    fps_difference = (
        current_fps_time
        - previous_fps_time
    )

    if fps_difference > 0:
        fps = 1 / fps_difference
    else:
        fps = 0

    previous_fps_time = current_fps_time

    cv2.putText(
        frame,
        f"FPS: {fps:.1f}",
        (20, height - 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2
    )

    cv2.imshow(
        "PID Tracking",
        frame
    )

    cv2.imshow(
        "Yellow Mask",
        mask
    )

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

camera.release()
arduino.close()
cv2.destroyAllWindows()