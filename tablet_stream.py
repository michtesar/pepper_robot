from robot import Pepper
import config

pepper = Pepper(config.IP_ADDRESS, config.PORT)
pepper.share_localhost("/Users/michael/Desktop/Pepper/tmp/")
pepper.subscribe_camera("camera_top", 2, 30)

while True:
    pepper.show_tablet_camera("Camera Top")
