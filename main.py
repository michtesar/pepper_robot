from robot import *
import time

pepper = Pepper("pepper.local")

#pepper.stat_animation("Hey_1")


"""
print("[PIPELINE]: Setting the robot")
pepper.set_volume(30)
pepper.set_security_distance(0.01)
pepper.tablet_show_web("https://www.ciirc.cvut.cz/cs/")

print("[PIPELINE]: Start exploration")
pepper.exploration_mode(1)
pepper.robot_localization()
pepper.show_map(on_robot=True, remote_ip="192.168.0.103")

print("[PIPELINE]: Navigate to 1.0")
pepper.navigate_to(1.0, 0.0)
pepper.robot_localization()

print("[PIPELINE]: Navigate to 2.0")
pepper.navigate_to(2.0, 0.0)
pepper.robot_localization()

print("[PIPELINE]: Navigate to -1.0")
pepper.navigate_to(-1.0, 0.0)
pepper.robot_localization()

print("[PIPELINE]: Navigate to -2.0")
pepper.navigate_to(-2.0, 0.0)
pepper.robot_localization()

print("[PIPELINE]: Pick a volunteer")
pepper.pick_a_volunteer()

print("[PIPELINE]: Play sound")
pepper.play_sound("/home/nao/song.mp3")
time.sleep(20)

# TODO: Face recognition - age - sex - mood detection
# TODO: Party mode on
"""
