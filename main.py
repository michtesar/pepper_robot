from robot import *
import cv2

pepper = Pepper("paprika.local")
#pepper.set_security_distance(0.08)
#pepper.exploration_mode(4)
#pepper.load_map("2018-05-04T101342.373Z.explo")
#pepper.robot_localization()
#pepper.navigate_to(-0.7, 0.0)
#pepper.show_map()
#pepper.come_here()
#pepper.restart_robot()
#pepper.stand()

#image = pepper.get_camera("camera_top", 1)
#pepper.stream_camera("camera_top", 1)
image = pepper.get_camera_frame()
print image

#pepper.set_security_distance(0.1)
#pepper.exploration_mode(3)
#pepper.load_map("2018-05-04T101342.373Z.explo")
#pepper.robot_localization()
#pepper.show_map()
