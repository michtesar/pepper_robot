"""
PEPPER ROBOT

This code implements a real robot controller based on several
qi services and it also includes a virtual robot mainly for
camera and speech recognition testing purposes.

Copyright (c) CTU in Prague  - All Rights Reserved
Created on: 3.5.2018
    Author: Michael Tesar <michael.tesar@cvut.cz>

Plymouth, United Kingdom 2018
"""
import qi
import time
import numpy
import cv2
import speech_recognition
import gtts
import playsound
import subprocess


class Pepper:
    """
    Real robot controller

    Specify a address of the robot
        - by hostname
        - by IP address

    Port is usually the same, e.g 9559 and it is not
    mandatory to specify.

    Example: pepper = Pepper("paprika.local")
    """
    def __init__(self, ip_address, port=9559):
        self.session = qi.Session()
        self.session.connect("tcp://" + ip_address + ":" + str(port))

        self.posture_service = self.session.service("ALRobotPosture")
        self.motion_service = self.session.service("ALMotion")
        self.tracker_service = self.session.service("ALTracker")
        self.tts_service = self.session.service("ALAnimatedSpeech")
        self.tablet_service = self.session.service("ALTabletService")
        self.autonomous_life_service = self.session.service("ALAutonomousLife")
        self.system_service = self.session.service("ALSystem")
        self.navigation_service = self.session.service("ALNavigation")
        self.battery_service = self.session.service("ALBattery")
        self.awareness_service = self.session.service("ALBasicAwareness")
        self.led_service = self.session.service("ALLeds")
        self.audio_device = self.session.service("ALAudioDevice")
        self.camera_device = self.session.service("ALVideoDevice")
        self.face_detection_service = self.session.service("ALFaceDetection")
        self.memory_service = self.session.service("ALMemory")

        self.slam_map = None
        self.localization = None
        self.camera_link = None

    def stand(self):
        """Get robot into default standing position"""
        self.posture_service.goToPosture("Stand", 0.5)

    def rest(self):
        """Get robot into default resting position"""
        self.posture_service.goToPosture("Crouch", 0.5)

    def point_at(self, x, y, z, effector_name, frame):
        """
        Point to some cartesian space coordinates
        by selected end-effector

        :param frame: in which is relative to
            0: Torso
            1: World
            2: Robot
        :param x: x axis in meters
        :param y: y axis in meters
        :param z: z axis in meters
        :param effector_name: LArm, RArm or Arms
        """
        speed = 0.5     # 50 %

        self.tracker_service.pointAt(effector_name, [x, y, z], frame, speed)

    def move_forward(self, distance):
        self.motion_service.move(distance, 0, 0)

    def turn_around(self, theta):
        self.motion_service.move(0, 0, theta)

    def stop_moving(self):
        self.motion_service.stopMove()

    def say(self, text):
        """Text to speech (robot internal engine)"""
        self.tts_service.say(text)

    def tablet_show_web(self, url):
        """
        Show web page on robot tablet

        Example:
            pepper.tablet_show_web("http://192.168.0.102:8000/keyboard.png")

        Start server:
            python -m SimpleHTTPServer
        """
        # FIXME: It is now working on Paprika
        self.tablet_service.turnScreenOn(True)
        self.tablet_service.showWebview()
        self.tablet_service.loadUrl(url)

    def tablet_show_image(self, image_url):
        """Show image from URL on robot tablet"""
        # FIXME: It is now working on Paprika
        self.tablet_service.showImage(image_url)

    def tablet_show_settings(self):
        """Show setting on the tablet"""
        self.tablet_service.showWebview("http://198.18.0.1/")

    def restart_robot(self):
        """Restart robot (it takes several minutes)"""
        self.system_service.reboot()

    def shutdown_robot(self):
        """Turn off the robot"""
        self.system_service.shutdown()

    def autonomous_life_off(self):
        """Switch autonomous life off and get into default position"""
        self.autonomous_life_service.setState("disabled")
        self.stand()

    def autonomous_life_on(self):
        """Switch autonomous life on"""
        self.autonomous_life_service.setState("interactive")

    def track_object(self, object_name, effector_name, diameter=0.05):
        """
        Track a object with a given object type and diameter.

        :param object_name: RedBall, Face, LandMark, LandMarks, People, Sound
        :param effector_name: LArm, RArm, Arms
        :param diameter: For face is default 15 cm, for ball 5 cm
        """
        if object == "Face":
            self.tracker_service.registerTarget(object_name, 0.15)
        else:
            self.tracker_service.registerTarget(object_name, diameter)

        self.tracker_service.setMode("Move")
        self.tracker_service.track(object_name)
        self.tracker_service.setEffector(effector_name)

        self.say("Show me a " + object_name)
        print("Use Ctrl+c to stop tracking")

        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("Interrupted by user")
            self.say("Stopping to track a " + object_name)

        self.tracker_service.stopTracker()
        self.unsubscribe_effector()
        self.say("Let's do something else!")

    def exploration_mode(self, radius):
        """
        Start exploration mode when robot it performing a SLAM
        in specified radius

        :param radius: distance in meters
        :return: image of the map
        """
        self.say("Starting exploration in " + str(radius) + " meters")
        self.navigation_service.explore(radius)
        map_file = self.navigation_service.saveExploration()

        print("[INFO]: Map file " + map_file)

        self.navigation_service.startLocalization()
        self.navigation_service.navigateToInMap([0., 0., 0.])
        self.navigation_service.stopLocalization()

        # Retrieve and display the map built by the robot
        result_map = self.navigation_service.getMetricalMap()
        map_width = result_map[1]
        map_height = result_map[2]
        img = numpy.array(result_map[4]).reshape(map_width, map_height)
        img = (100 - img) * 2.55  # from 0..100 to 255..0
        img = numpy.array(img, numpy.uint8)

        self.slam_map = img

    def show_map(self, on_robot=False, remote_ip=None):
        result_map = self.navigation_service.getMetricalMap()
        map_width = result_map[1]
        map_height = result_map[2]
        img = numpy.array(result_map[4]).reshape(map_width, map_height)
        img = (100 - img) * 2.55  # from 0..100 to 255..0
        img = numpy.array(img, numpy.uint8)

        resolution = result_map[0]

        self.robot_localization()

        offset_x = result_map[3][0]
        offset_y = result_map[3][1]
        x = self.localization[0]
        y = self.localization[1]

        goal_x = (x - offset_x) / resolution
        goal_y = -1 * (y - offset_y) / resolution

        img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        cv2.circle(img, (int(goal_x), int(goal_y)), 3, (0, 0, 255), -1)

        robot_map = cv2.resize(img, None, fx=1, fy=1, interpolation=cv2.INTER_CUBIC)

        print("[INFO]: Showing the map")

        if on_robot:
            cv2.imwrite("./tmp/map.png", robot_map)
            self.tablet_show_web(remote_ip + ":8000/map.png")
        else:
            cv2.imshow("RobotMap", robot_map)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    def robot_localization(self):
        try:
            self.navigation_service.startLocalization()
            localization = self.navigation_service.getRobotPositionInMap()
            self.localization = localization[0]
            print("[INFO]: Localization complete")
            self.navigation_service.stopLocalization()
        except:
            print("[INFO]: Localization failed")

    def stop_localization(self):
        self.navigation_service.stopLocalization()

    def load_map(self, file_name, file_path="/home/nao/.local/share/Explorer/"):
        """Load a previsouly generated map which by default is stored on shared folder"""
        self.slam_map = self.navigation_service.loadExploration(file_path + file_name)
        print("[INFO]: Map '" + file_name + "' loaded")

    def set_volume(self, volume):
        """Set robot volume in 0 - 100 %"""
        self.audio_device.setOutputVolume(volume)
        self.say("Volume is set to " + str(volume) + " percent")

    def battery_status(self):
        """Say a battery status"""
        battery = self.battery_service.getBatteryCharge()
        self.say("I have " + str(battery) + " percent of battery")

    def set_awareness(self, state):
        """
        Turn on or off the basic awareness of the robot,
        e.g. looking for humans, self movements etc.

        :param state: boolean (True/False)

        Example:

            pepper.set_awareness(False) to turn off awareness
        """
        if state:
            self.awareness_service.resumeAwareness()
        else:
            self.awareness_service.pauseAwareness()

    def subscribe_camera(self, camera, resolution, fps):
        """
        Subscribe to a camera service

        :param fps: 5, 10, 15 or 30
        :param camera: camera_top, camera_bottom
        :param resolution:
            0: 160x120
            1: 320x240
            2: 640x480
            3: 1280x960
        """
        camera_index = None
        if camera == "camera_top":
            camera_index = 0
        elif camera == "camera_bottom":
            camera_index = 1

        self.camera_link = self.camera_device.subscribeCamera("Camera_Stream" + str(numpy.random),
                                                              camera_index, resolution, 13, fps)

    def unsubscribe_camera(self):
        """Unsubscribe to camera"""
        self.camera_device.unsubscribe(self.camera_link)

    def get_camera_frame(self, show):
        """
        Get camera frame from subscribed camera link

        :param show: Show image with OpenCV
        :return: image (array)
        """
        image_raw = self.camera_device.getImageRemote(self.camera_link)
        image = numpy.frombuffer(image_raw[6], numpy.uint8).reshape(image_raw[1], image_raw[0], 3)

        if show:
            cv2.imshow("Pepper Camera", image)
            cv2.waitKey(-1)
            cv2.destroyAllWindows()

        return image

    def set_security_distance(self, distance=0.05):
        """Set security distance. Lower distance for passing doors etc."""
        self.motion_service.setOrthogonalSecurityDistance(distance)
        print("[INFO]: Security distance set to " + str(distance) + " m")

    def move_head_down(self):
        """Look down"""
        self.motion_service.setAngles("HeadPitch", 0.4, 0.2)

    def move_head_up(self):
        """Look up"""
        self.motion_service.setAngles("HeadPitch", -0.4, 0.2)

    def move_head_default(self):
        """Put head into default position"""
        self.motion_service.setAngles("HeadPitch", 0.0, 0.2)

    def move_to_circle(self, clockwise, t=10):
        """
        Moving robot into circle

        :param clockwise: boolean (True/False)
        :param t: time in seconds
        """
        if clockwise:
            self.motion_service.moveToward(0.5, 0.0, 0.6)
        else:
            self.motion_service.moveToward(0.5, 0.0, -0.6)
        time.sleep(t)
        self.motion_service.stopMove()

    def blink_eyes(self, rgb):
        """Blink eyes with defined color"""
        # FIXME: It is not working on Paprika
        self.led_service.fadeRGB('AllLeds', rgb[0], rgb[1], rgb[2], 1.0)

    def navigate_to(self, x, y):
        """
        Navigate robot in map based on exploration mode
        or load previsouly mapped enviroment

        :param x: x axis in meters
        :param y: y axis in meters
        """
        try:
            self.navigation_service.startLocalization()
            self.navigation_service.navigateToInMap([x, y, 0])
            self.navigation_service.stopLocalization()

            self.say("At your command")
        except:
            self.say("I cannot move in that direction")

    def unsubscribe_effector(self):
        self.tracker_service.unregisterAllTargets()
        self.tracker_service.setEffector("None")

    def pick_a_volunteer(self):
        volunteer_found = False
        self.unsubscribe_effector()
        self.stand()
        self.say("I need a volunteer.")

        proxy_name = "FaceDetection" + str(numpy.random)

        while not volunteer_found:
            wait = numpy.random.randint(500, 1500) / 1000
            theta = numpy.random.randint(-5, 5)
            self.turn_around(theta)
            time.sleep(wait)
            self.stop_moving()
            self.stand()
            self.face_detection_service.subscribe(proxy_name, 500, 0.0)

            for memory in range(2):
                time.sleep(0.5)
                output = self.memory_service.getData("FaceDetected")
                print("...")
                if output and isinstance(output, list) and len(output) >= 2:
                    print("Face detected")
                    volunteer_found = True

        self.say("I found a volunteer! It is you!")
        self.stand()
        self.tracker_service.registerTarget("Face", 0.15)
        self.tracker_service.setMode("Move")
        self.tracker_service.track("Face")
        self.tracker_service.setEffector("RArm")

        time.sleep(2)

        self.unsubscribe_effector()
        self.stand()
        self.face_detection_service.unsubscribe(proxy_name)

    def share_localhost(self, folder):
        subprocess.Popen(["cd", folder])
        subprocess.Popen(["python", "-m", "SimpleHTTPServer"])


class VirtualPepper:
    """Virtual robot for testing"""

    def __init__(self):
        """Constructor of virtual robot"""
        print("[INFO]: Using virtual robot!")

    @staticmethod
    def say(text):
        """Say some text trough text to speech"""
        tts = gtts.gTTS(text, lang="en")
        tts.save("./tmp_speech.mp3")
        playsound.playsound("./tmp_speech.mp3")

    @staticmethod
    def listen():
        """Speech to text by Google Speech Recognition"""
        recognizer = speech_recognition.Recognizer()
        with speech_recognition.Microphone() as source:
            print("[INFO]: Say something...")
            audio = recognizer.listen(source)
            speech = recognizer.recognize_google(audio, language="en-US")

            return speech

    @staticmethod
    def stream_camera():
        """Stream web camera of the computer (if any)"""
        print("[INFO]: Press q to quit camera stream")

        cap = cv2.VideoCapture(0)

        while True:
            ret, frame = cap.read()
            cv2.imshow("frame", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        cap.release()
        cv2.destroyAllWindows()

    @staticmethod
    def camera_image():
        """Show one frame from web camera (if any)"""
        cap = cv2.VideoCapture(0)
        while True:
            ret, img = cap.read()
            cv2.imshow("input", img)

            key = cv2.waitKey(10)
            if key == 27:
                break

        cv2.destroyAllWindows()
        cv2.VideoCapture(0).release()
