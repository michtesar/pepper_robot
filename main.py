from robot import *
from time import sleep


pepper = Pepper("192.168.0.101")
pepper.set_volume(45)

while True:
    pepper.say("Give me a question")
    try:
        pepper.ask_wikipedia()
    except Exception as error:
        print(error)
        pepper.say("I am not sure what to say")
