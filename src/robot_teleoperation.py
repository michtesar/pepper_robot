from XboxController import XboxController
import time
import sys
from robot import Pepper

pepper = Pepper("192.168.0.101")
#pepper.set_security_distance(0.0)
pepper.autonomous_life_off()

def controlCallBack(xboxControlId, value):
    pass


def go_forward(value):
    pepper.move_forward(value)

def go_backward(value):
    pepper.move_forward(-value)

def leftThumbX(xValue):
    print "LX {}".format(xValue)
    pepper.turn_around(-xValue)


def leftThumbY(yValue):
    print "LY {}".format(yValue)
    pepper.move_forward(yValue)

def say_yes(aValue):
    if aValue == 1:
        pepper.say("yes")

def say_no(aValue):
    if aValue == 1:
        pepper.say("no")

def say_hello(aValue):
    if aValue == 1:
        pepper.say("hello")

def say_dont_know(aValue):
    if aValue == 1:
        pepper.say("i dont know")

def test(aValue):
    print aValue

xboxCont = XboxController(controlCallBack, deadzone=30, scale=100, invertYAxis=True)

xboxCont.setupControlCallback(xboxCont.XboxControls.LTHUMBX, leftThumbX)
xboxCont.setupControlCallback(xboxCont.XboxControls.LTHUMBY, leftThumbY)
xboxCont.setupControlCallback(xboxCont.XboxControls.A, say_yes)
xboxCont.setupControlCallback(xboxCont.XboxControls.B, say_no)
xboxCont.setupControlCallback(xboxCont.XboxControls.X, say_hello)
xboxCont.setupControlCallback(xboxCont.XboxControls.Y, say_dont_know)
xboxCont.setupControlCallback(xboxCont.XboxControls.LTRIGGER, go_forward)
xboxCont.setupControlCallback(xboxCont.XboxControls.RTRIGGER, go_backward)

try:
    xboxCont.start()
    print "xbox controller running"
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print "User cancelled"
except:
    print "Unexpected error:", sys.exc_info()[0]
    raise
finally:
    xboxCont.stop()
