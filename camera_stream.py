from robot import Pepper
import cv2
import config

pepper = Pepper(config.IP_ADDRESS, config.PORT)
pepper.subscribe_camera("camera_top", 2, 30)

while True:
    image = pepper.get_camera_frame(show=False)
    cv2.imshow("frame", image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

pepper.unsubscribe_camera()
cv2.destroyAllWindows()
