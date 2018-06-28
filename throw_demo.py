from robot import Pepper

pepper = Pepper("10.37.1.227")
speed = 0.1

pepper.stand()

# Raise a hand to human
pepper.motion_service.angleInterpolationWithSpeed(["RShoulderPitch", "RWristYaw", "RHand"], [0.8, 2.5, 1.0], speed)

# Wait to touch a hand
while True:
    try:
        status = pepper.memory_service.getData("HandRightBackTouched")
        if status:
            pepper.motion_service.angleInterpolationWithSpeed("RHand", 0.0, speed)
            break
    except KeyboardInterrupt:
        break

# Get hand down
pepper.motion_service.angleInterpolationWithSpeed(["RShoulderPitch", "RWristYaw", "RHand"], [3.5, 0.00, 0.0], speed)

# Throw a ball
pepper.motion_service.angleInterpolationWithSpeed(["RShoulderPitch", "RWristYaw"], [1.0, 2.5], speed)
pepper.motion_service.angleInterpolationWithSpeed("RHand", 1.0, speed)

# Reset position
pepper.stand()

