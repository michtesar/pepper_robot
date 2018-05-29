from robot import Pepper
import ConfigParser

config = ConfigParser.ConfigParser()
config.read("config.ini")

pepper = Pepper(config.get("DEFAULT", "IP_ADDRESS"))
pepper.share_localhost("/Users/michael/Desktop/Pepper/tmp/")
pepper.subscribe_camera("camera_top", 2, 30)

while True:
    pepper.show_tablet_camera("Camera Top")
