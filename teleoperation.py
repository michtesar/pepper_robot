from robot import Pepper
import cv2

pepper = Pepper("paprika.local")
pepper.set_security_distance(0.5)

image = cv2.imread("keyboard.png")
height, width = image.shape[:2]

cv2.namedWindow("Robot teleoperation", cv2.WINDOW_GUI_NORMAL)
cv2.resizeWindow("Robot teleoperation", width, height)

while True:
    cv2.imshow("Robot teleoperation", image)
    k = cv2.waitKeyEx()
    if k == 27:
        break

    if k == 100:
        # Turn right
        pepper.turn_around(-0.5)
    elif k == 119:
        # Go forward
        pepper.move_forward(0.5)
    elif k == 97:
        # Turn left
        pepper.turn_around(1.0)
    elif k == 115:
        # Go back
        pepper.move_forward(1.0)
    elif k == 32:
        # Stop moving
        pepper.stop_moving()

