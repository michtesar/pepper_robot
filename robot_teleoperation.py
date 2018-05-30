import pygame
import random
import robot
import sys
import subprocess
import time
import config

pygame.init()
j = pygame.joystick.Joystick(0)
j.init()

left_x = 0
left_y = 0

right_x = 0
right_y = 0

left_trigger = 0
right_trigger = 0


button_names = ["A", "B", "X", "Y", "LB", "RB", "BACK", "START", "MENU", "Left", "Right"]
menu = ["battery status", "pick a volunteer", "dance", "ask wikipedia", "blink my eyes", "show website",
        "show image", "rest", "get face properties", "about me"]
menu_functions = ["pepper.battery_status()", "pepper.pick_a_volunteer()", "pepper.start_dance()",
                  "pepper.ask_wikipedia()", "blink_eyes()", "pepper.tablet_show_web('https://www.ciirc.cvut.cz')",
                  "pepper.tablet_show_image('https://goo.gl/4Xq6Bc')", "pepper.rest()",
                  "pepper.get_face_properties()", "pepper.say('My name is Cinnamon. I am circ robot. I was designed as social robot by Softbanks robotics. At circ my colegues Gabriela, Michael and Michal develop artificial intelligence with me. They will show you. Luckily.')"]
menu_index = -1
menu_items = len(menu)
hat = None
button = 0

pepper = robot.Pepper(config.IP_ADDRESS, config.PORT)
pepper.set_security_distance(0.01)


def blink_eyes():
    pepper.blink_eyes([255, 0, 0])
    time.sleep(2)
    pepper.blink_eyes([0, 0, 0])


def switch_al():
    status = pepper.autonomous_life_service.getState()
    if status == "disabled":
        pepper.say("Autonomous life on")
        pepper.autonomous_life_on()
    else:
        pepper.say("Autonomous life off")
        pepper.autonomous_life_off()


try:
    while True:
        events = pygame.event.get()
        for event in events:
            if event.type == pygame.JOYAXISMOTION:
                # Left axis (0, 1)
                if event.axis == 0:
                    if event.value > 0.2 or event.value < -0.2:
                        left_x = event.value
                    else:
                        left_x = 0
                if event.axis == 1:
                    if event.value > 0.2 or event.value < -0.2:
                        left_y = event.value
                    else:
                        left_y = 0
                # Right axis (3, 4)
                if event.axis == 3:
                    if event.value > 0.2 or event.value < -0.2:
                        right_x = event.value
                    else:
                        right_x = 0
                if event.axis == 4:
                    if event.value > 0.1 or event.value < -0.1:
                        right_y = event.value
                    else:
                        right_y = 0
                # Left and right triggers (2, 5)
                if event.axis == 2:
                    if event.value > 0:
                        left_trigger = event.value
                    else:
                        left_trigger = 0
                if event.axis == 5:
                    if event.value > 0:
                        right_trigger = event.value
                    else:
                        right_trigger = 0

                if right_trigger:
                    pepper.move_forward(right_trigger)
                else:
                    pepper.move_forward(-left_trigger)

                if left_x:
                    pepper.turn_around(-left_x)

                if right_x:
                    pepper.motion_service.setAngles("HeadYaw", -right_x, 0.3)
                if right_y:
                    pepper.motion_service.setAngles("HeadPitch", right_y, 0.3)

            elif event.type == pygame.JOYBALLMOTION:
                print(event.dict, event.joy, event.ball, event.rel)
            elif event.type == pygame.JOYBUTTONDOWN:
                button = button_names[event.button]
                if button == "MENU":
                    switch_al()
                elif button == "Y":
                    pepper.say("yes")
                elif button == "A":
                    pepper.say("no")
                elif button == "X":
                    pepper.say("i dont know")
                elif button == "B":
                    pepper.say("hello")
                elif button == "BACK":
                    pepper.stand()
                elif button == "START":
                    animation = ["Hey_1", "Hey_2", "Hey_3", "Hey_4"]
                    pepper.start_animation(random.choice(animation))
                    pepper.stand()
                elif button == "LB":
                    s = subprocess.Popen([sys.executable, '-c', 'from robot import Pepper; pepper = Pepper("192.168.8.101"); pepper.play_sound("/home/nao/song.mp3")'],
                                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
                    d = subprocess.Popen([sys.executable, '-c',
                                          'from robot import Pepper; pepper = Pepper("192.168.8.101"); pepper.start_dance()'],
                                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
                elif button == "RB":
                    p = subprocess.Popen([sys.executable, '-c',
                                          'from robot import Pepper; pepper = Pepper("192.168.8.101"); pepper.stop_sound()'],
                                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
                elif button == "Left":
                    pepper.hand("right", True)
                elif button == "Right":
                    pepper.hand("right", False)
            elif event.type == pygame.JOYHATMOTION:
                hat = event.value
                x = hat[0]
                y = hat[1]
                menu_index += y
                try:
                    if y != 0 and hat != (0, 0):
                        action = (menu[menu_index])
                        pepper.say(action)
                except IndexError:
                    menu_index = 0

                if x == 1 and hat != (0, 0):
                    ok = ["ok", "roger that", "will do", "all right"]
                    pepper.say(random.choice(ok))
                    eval(menu_functions[menu_index])

                if x == -1 and hat != (0, 0):
                    pepper.point_at(1.0, 1.0, 0.5, "RArm", 0)

except KeyboardInterrupt:
    print("EXITING NOW")
    j.quit()
