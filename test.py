import cv2 as cv

camera = cv.VideoCapture(1)

if not camera.isOpened():
    print("Could not open camera")
    input("Press Enter to exit...")
    exit()

while True:
    success, frame = camera.read()

    if not success:
        print("Could not read frame")
        break

    cv.imshow("Webcam Test", frame)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

camera.release()
cv.destroyAllWindows()

input("Press Enter to exit...")