from robot import Pepper

pepper = Pepper("10.37.1.227")
pepper.share_localhost("/Users/michael/Desktop/Pepper/tmp/")
pepper.subscribe_camera("camera_top", 2, 30)

while True:
    pepper.show_tablet_camera("Camera Top")
