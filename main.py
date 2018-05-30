from robot import *
import config

pepper = Pepper(config.IP_ADDRESS, config.PORT)

while True:
    pepper.say("Give me a question")
    try:
        pepper.ask_wikipedia()
    except Exception as error:
        print(error)
        pepper.say("I am not sure what to say")
