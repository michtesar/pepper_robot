from robot import Pepper
import cv2

pepper = Pepper("10.37.1.227")
pepper.set_security_distance(0.01)

image = cv2.imread("src/keyboard.png")
height, width = image.shape[:2]

cv2.namedWindow("Robot teleoperation", cv2.WINDOW_GUI_NORMAL)
cv2.resizeWindow("Robot teleoperation", width, height)

while True:
    cv2.imshow("Robot teleoperation", image)
    k = cv2.waitKeyEx()
    # ESC - quit application
    if k == 27:
        break

    # D - turn right
    if k == 100:
        print("[INFO]: Turn right")
        pepper.turn_around(-10)
    # W - go forward
    elif k == 119:
        print("[INFO]: Go forward")
        pepper.move_forward(1000)
    # A - turn left
    elif k == 97:
        print("[INFO]: Turn left")
        pepper.turn_around(10)
    # S - go backward
    elif k == 115:
        print("[INFO]: Go backward")
        pepper.move_forward(-1000)
    # SPACE - stop movement
    elif k == 32:
        print("[INFO]: Stop movement")
        pepper.stop_moving()
    # Q - autonomous life on
    elif k == 113:
        pepper.autonomous_life_on()
    # E - autonomous life off
    elif k == 101:
        pepper.autonomous_life_off()
    # R - awareness on
    elif k == 114:
        pepper.set_awareness(True)
    # T - awareness off
    elif k == 116:
        pepper.set_awareness(False)
    # Y - stop exploration
    elif k == 121:
        print("[INFO]: Stopping exploration")
        pepper.navigation_service.stopExploration()
