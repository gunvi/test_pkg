import cv2
import time
import random

def capture_image():
    cam = cv2.VideoCapture(0)
    ret, frame = cam.read()
    filename = None

    if ret:
        t = time.strftime('%Y%m%d_%H%M%S')
        r = random.randint(1000, 9999)
        filename = f"/home/vg88/servo_project/captured_{t}_{r}.jpg"
        cv2.imwrite(filename, frame)
        print(f"Image saved as {filename}")

    cam.release()
    cv2.destroyAllWindows()
    return filename
