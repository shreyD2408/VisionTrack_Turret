import cv2
import numpy as np
import serial
import time

# =========================
# SETTINGS
# =========================

SERIAL_PORT = "COM3"
BAUD_RATE = 9600
CAMERA_INDEX = 1

# Yellow HSV range
LOWER_YELLOW = np.array([18, 70, 50])
UPPER_YELLOW = np.array([40, 255, 255])

# Minimum yellow object size
MIN_AREA = 100

# Ignore extremely wide/tall detections
MIN_ASPECT_RATIO = 0.2
MAX_ASPECT_RATIO = 5.0

# How much the target can move before updating
# the servo position
SERIAL_INTERVAL = 0.03

# How much smoothing to apply
SMOOTHING = 0.25


# =========================
# SERIAL
# =========================

arduino = serial.Serial(
    SERIAL_PORT,
    BAUD_RATE,
    timeout=1
)

time.sleep(2)


# =========================
# CAMERA
# =========================

camera = cv2.VideoCapture(CAMERA_INDEX)

if not camera.isOpened():
    print("Could not open camera")
    arduino.close()
    exit()


# Try to use 640x480
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)


# =========================
# VARIABLES
# =========================

smoothed_x = None
last_send_time = 0
last_detection_time = time.time()

previous_time = time.time()


# =========================
# MAIN LOOP
# =========================

while True:

    success, frame = camera.read()

    if not success:
        print("Could not read camera")
        break


    # -------------------------
    # CAMERA INFORMATION
    # -------------------------

    height, width = frame.shape[:2]

    center_x = width // 2
    center_y = height // 2


    # -------------------------
    # CONVERT TO HSV
    # -------------------------

    hsv = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2HSV
    )


    # -------------------------
    # CREATE YELLOW MASK
    # -------------------------

    mask = cv2.inRange(
        hsv,
        LOWER_YELLOW,
        UPPER_YELLOW
    )


    # -------------------------
    # CLEAN MASK
    # -------------------------

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


    # -------------------------
    # FIND CONTOURS
    # -------------------------

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )


    best_contour = None
    best_area = 0


    # -------------------------
    # FIND BEST TARGET
    # -------------------------

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


        # Choose the largest valid yellow object
        if area > best_area:

            best_area = area
            best_contour = contour


    # -------------------------
    # PROCESS TARGET
    # -------------------------

    if best_contour is not None:

        x, y, w, h = cv2.boundingRect(
            best_contour
        )

        object_x = x + w // 2
        object_y = y + h // 2


        # -------------------------
        # SMOOTH X POSITION
        # -------------------------

        if smoothed_x is None:

            smoothed_x = object_x

        else:

            smoothed_x = (
                SMOOTHING * object_x
                + (1 - SMOOTHING) * smoothed_x
            )


        smoothed_x = int(smoothed_x)


        # -------------------------
        # SEND TO ARDUINO
        # -------------------------

        current_time = time.time()

        if (
            current_time - last_send_time
            >= SERIAL_INTERVAL
        ):

            arduino.write(
                f"{smoothed_x}\n".encode()
            )

            last_send_time = current_time


        last_detection_time = current_time


        # -------------------------
        # DRAW TARGET
        # -------------------------

        cv2.rectangle(
            frame,
            (x, y),
            (x + w, y + h),
            (0, 255, 0),
            2
        )


        cv2.circle(
            frame,
            (smoothed_x, object_y),
            6,
            (0, 0, 255),
            -1
        )


        # Target information

        cv2.putText(
            frame,
            f"X: {smoothed_x}",
            (x, y - 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2
        )

        cv2.putText(
            frame,
            f"Area: {int(best_area)}",
            (x, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2
        )


    else:

        # No target detected

        cv2.putText(
            frame,
            "TARGET NOT FOUND",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2
        )


    # -------------------------
    # CAMERA CENTER
    # -------------------------

    cv2.line(
        frame,
        (center_x, 0),
        (center_x, height),
        (255, 0, 0),
        2
    )


    # Draw center point

    cv2.circle(
        frame,
        (center_x, center_y),
        5,
        (255, 0, 0),
        -1
    )


    # -------------------------
    # FPS
    # -------------------------

    current_time = time.time()

    fps = 1 / (
        current_time - previous_time
    )

    previous_time = current_time


    cv2.putText(
        frame,
        f"FPS: {fps:.1f}",
        (20, height - 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2
    )


    # -------------------------
    # DISPLAY
    # -------------------------

    cv2.imshow(
        "Yellow Object Tracking",
        frame
    )

    cv2.imshow(
        "Yellow Mask",
        mask
    )


    # -------------------------
    # QUIT
    # -------------------------

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break


# =========================
# CLEANUP
# =========================

camera.release()

arduino.close()

cv2.destroyAllWindows()