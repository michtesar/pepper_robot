from robot import *
import ConfigParser

config = ConfigParser.ConfigParser()
config.read("config.ini")

pepper = Pepper(config.get("DEFAULT", "IP_ADDRESS"))

while True:
    pepper.say("Give me a question")
    try:
        pepper.ask_wikipedia()
    except Exception as error:
        print(error)
        pepper.say("I am not sure what to say")
