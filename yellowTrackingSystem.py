import cv2 as cv
import numpy as np

camera = cv.VideoCapture(1)

if not camera.isOpened():
    print("Could not open camera")
    input("Press Enter to exit...")
    exit()
    
width = camera.get(cv.CAP_PROP_FRAME_WIDTH)
height = camera.get(cv.CAP_PROP_FRAME_HEIGHT)
print(width, "x", height)

while True:
    success, frame = camera.read()

    if not success:
        print("Could not read frame")
        break

    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
  
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])

    mask = cv.inRange(hsv, lower_yellow, upper_yellow)

    contours, _ = cv.findContours(
        mask,
        cv.RETR_EXTERNAL,
        cv.CHAIN_APPROX_NONE
    )

    for contour in contours:

        area = cv.contourArea(contour)

        if area > 500:

            x, y, w, h = cv.boundingRect(contour)

            center_x = x + w // 2
            center_y = y + h // 2

            cv.rectangle(
                frame,
                (x, y),
                (x + w, y + h),
                (0, 255, 255),
                2
            )

            cv.circle(
                frame,
                (center_x, center_y),
                5,
                (0, 0, 255),
                -1
            )

            cv.putText(
                frame,
                f"X: {center_x}, Y: {center_y}",
                (x, y - 10),
                cv.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

    cv.imshow("Yellow Object Tracking", frame)

    if cv.waitKey(1) & 0xFF == ord("q"):
        break

camera.release()
cv.destroyAllWindows()