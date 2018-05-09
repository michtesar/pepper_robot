"""
PEPPER ROBOT API for Python
===========================

This code implements a real robot controller based on several
qi services and it also includes a virtual robot mainly for
camera and speech recognition testing purposes.

Copyright (c) CTU in Prague  - All Rights Reserved

Created on: 3.5.2018 Plymouth, United Kingdom

Author: Michael Tesar

<michael.tesar@cvut.cz>
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
    **Real robot controller**

    Specify a address of the robot by:

    - hostname (*Avahai* style hostname like `pepper.local`)
    - IP address (can be obtained by pressing robot's *chest button*)

    Port is usually the same, e.g 9559 and it is not
    mandatory to specify.

    ```
    python
    pepper = Pepper("paprika.local")
    ```
    """
    def __init__(self, ip_address, port=9559):
        """
        Class constructor

        - `ip_address`: IP address of the robot
        - `port`: port, default 9559
        """
        self.session = qi.Session()
        self.session.connect("tcp://" + ip_address + ":" + str(port))

        self.posture_service = self.session.service("ALRobotPosture")
        '''Posture service for managing postures (like `StandInit` or `Rest`)'''
        self.motion_service = self.session.service("ALMotion")
        '''Service for all motion related tasks except navigation (it has separate service)'''
        self.tracker_service = self.session.service("ALTracker")
        '''Service for tracking objects such as red ball or face'''
        self.tts_service = self.session.service("ALAnimatedSpeech")
        '''Text to speech service with small movements while robot speaks'''
        self.tablet_service = self.session.service("ALTabletService")
        '''Tablet communication service'''
        self.autonomous_life_service = self.session.service("ALAutonomousLife")
        '''Service for turning on and off the autonomous life'''
        self.system_service = self.session.service("ALSystem")
        '''Core system service of the robot'''
        self.navigation_service = self.session.service("ALNavigation")
        '''Service for navigation and localization fo the robot'''
        self.battery_service = self.session.service("ALBattery")
        '''Battery status service'''
        self.awareness_service = self.session.service("ALBasicAwareness")
        '''Service of basic awareness which is part of autonomous life'''
        self.led_service = self.session.service("ALLeds")
        '''Service fot LEDs on the robot'''
        self.audio_device = self.session.service("ALAudioDevice")
        '''Service for audio capture'''
        self.camera_device = self.session.service("ALVideoDevice")
        '''Service for camera communication and image transfer'''
        self.face_detection_service = self.session.service("ALFaceDetection")
        '''Face detection service'''
        self.memory_service = self.session.service("ALMemory")
        '''Service for previously stored `memories`'''
        self.audio_service = self.session.service("ALAudioPlayer")
        '''Service for playing the music on the robot'''

        self.slam_map = None
        self.localization = None
        self.camera_link = None

        print("[INFO]: Robot is initialized at " + ip_address + ":" + port)

    def stand(self):
        """Get robot into default standing position known as `StandInit` or `Stand`"""
        self.posture_service.goToPosture("Stand", 0.5)
        print("[INFO]: Robot is in default position")

    def rest(self):
        """Get robot into default resting position know as `Crouch`"""
        self.posture_service.goToPosture("Crouch", 0.5)
        print("[INFO]: Robot is in resting position")

    def point_at(self, x, y, z, effector_name, frame):
        """
        Point to some cartesian space coordinates
        by selected end-effector

        - `frame`: in which is relative to
            - 0: Torso
            - 1: World
            - 2: Robot
        - `x`: X axis in meters
        - `y`: Y axis in meters
        - `z`: Z axis in meters
        - `effector_name`: One of the end-effectors:
            -LArm, RArm or Arms
        """
        speed = 0.5     # 50 % of speed
        self.tracker_service.pointAt(effector_name, [x, y, z], frame, speed)

    def move_forward(self, speed):
        """
        Move forward with certain speed

        - `speed`: Speed *(positive forward, negative backward)*
        """
        self.motion_service.move(speed, 0, 0)

    def turn_around(self, speed):
        """
        Turn around the robot by speed

        - `speed`: Speed of turning around
            - negative values turns to left
            - positive values turns to right
        """
        self.motion_service.move(0, 0, speed)

    def stop_moving(self):
        """Stop robot from moving by `move_around` and `turn_around` methods"""
        self.motion_service.stopMove()

    def say(self, text):
        """
        Text to speech (robot internal engine)

        - `text`: Text to speech
        """
        self.tts_service.say(text)
        print("[INFO]: Robot says: " + text)

    def tablet_show_web(self, url):
        """
        Show web page on robot tablet. It also works for
        sharing a locally stored images and websites to
        Pepper tablet by running:

            `pepper.share_localhost("~/Desktop/my_web_to_host/")`

        And then connects to in by:

            `pepper.tablet_show_web(<remote_ip>:8000/my_web_to_host/index.html")`

        Or you can start server manually:
            `python -m SimpleHTTPServer` (if empty default port is used: 8000)

        - `url`: Web URL
        """
        self.tablet_service.turnScreenOn(True)
        self.tablet_service.showWebview()
        self.tablet_service.loadUrl(url)

    def tablet_show_image(self, image_url):
        """
        Show image from URL: from outside or locally stored

        - `image_url`: Image URL
        """
        self.tablet_service.showImage(image_url)

    def tablet_show_settings(self):
        """Show robot settings on the tablet"""
        self.tablet_service.showWebview("http://198.18.0.1/")

    def restart_robot(self):
        """Restart robot (it takes several minutes)"""
        print("[WARN]: Restarting the robot")
        self.system_service.reboot()

    def shutdown_robot(self):
        """Turn off the robot completely"""
        print("[WARN]: Turning off the robot")
        self.system_service.shutdown()

    def autonomous_life_off(self):
        """Switch autonomous life off and get into default position"""
        self.autonomous_life_service.setState("disabled")
        self.stand()
        print("[INFO]: Autonomous life is off")

    def autonomous_life_on(self):
        """Switch autonomous life on"""
        self.autonomous_life_service.setState("interactive")
        print("[INFO]: Autonomous life is on")

    def track_object(self, object_name, effector_name, diameter=0.05):
        """
        Track a object with a given object type and diameter. If 'Face' is
        chosen it has a default parameters to 15 cm diameter per face. After
        staring tracking in will wait until user press ctrl+c.

        For more info about tracking modes, object names and other:
        http://doc.aldebaran.com/2-5/naoqi/trackers/index.html#tracking-modes

        - `object_name`: One of these: RedBall, Face, LandMark, LandMarks, People, Sound
        - `effector_name`: One of these: LArm, RArm, Arms
        - `diameter`: Diameter of the object (default 0.05, for face default 0.15)
        """
        if object == "Face":
            self.tracker_service.registerTarget(object_name, 0.15)
        else:
            self.tracker_service.registerTarget(object_name, diameter)

        self.tracker_service.setMode("Move")
        self.tracker_service.track(object_name)
        self.tracker_service.setEffector(effector_name)

        self.say("Show me a " + object_name)
        print("[INFO]: Use Ctrl+c to stop tracking")

        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("[INFO]: Interrupted by user")
            self.say("Stopping to track a " + object_name)

        self.tracker_service.stopTracker()
        self.unsubscribe_effector()
        self.say("Let's do something else!")

    def exploration_mode(self, radius):
        """
        Start exploration mode when robot it performing a SLAM
        in specified radius. Then it saves a map into robot into
        its default folder.

        - `radius`: Distance in meters

        Returns OpenCV image array
        """
        self.say("Starting exploration in " + str(radius) + " meters")
        self.navigation_service.explore(radius)
        map_file = self.navigation_service.saveExploration()

        print("[INFO]: Map file stored: " + map_file)

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
        """
        Shows a map from robot based on previously loaded one
        or explicit exploration of the scene. It can be viewed on
        the robot or in the computer by OpenCV.

        - `on_robot`: Show the map on a robot (default False)
        - `remote_ip`: IP address of remote localhost (default None)
        """
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
            # TODO: It requires a HTTPS server running. This should be somehow automated.
            cv2.imwrite("./tmp/map.png", robot_map)
            self.tablet_show_web(remote_ip + ":8000/map.png")
            print("[INFO]: Map is available at: " + str(remote_ip) + ":8000/map.png")
        else:
            cv2.imshow("RobotMap", robot_map)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    def robot_localization(self):
        """Localize a robot in a map which was loaded or after exploration"""
        try:
            self.navigation_service.startLocalization()
            localization = self.navigation_service.getRobotPositionInMap()
            self.localization = localization[0]
            print("[INFO]: Localization complete")
            self.navigation_service.stopLocalization()
        except:
            print("[ERROR]: Localization failed")

    def stop_localization(self):
        """Stop localization of the robot"""
        self.navigation_service.stopLocalization()
        print("[INFO]: Localization stopped")

    def load_map(self, file_name, file_path="/home/nao/.local/share/Explorer/"):
        """
        Load stored map on a robot. It will find a map in default location,
        in other cases alternative path can be specifies by `file_name`.

        - `file_name`: (string) Name of the map
        - `file_path`: (string) Path to the map with '/' at the end (points at default path)
        """
        self.slam_map = self.navigation_service.loadExploration(file_path + file_name)
        print("[INFO]: Map '" + file_name + "' loaded")

    def set_volume(self, volume):
        """
        Set robot volume in 0 - 100 %

        - `volume`: Volume of the robot
        """
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

        - `state`: If True set on, if False set off
        """
        if state:
            self.awareness_service.resumeAwareness()
            print("[INFO]: Awareness is turned on")
        else:
            self.awareness_service.pauseAwareness()
            print("[INFO]: Awareness is paused")

    def subscribe_camera(self, camera, resolution, fps):
        """
        Subscribe to a camera service. You need to subscribe a camera
        before you reach a images from it. Each subscription has to have
        a unique name otherwise it will conflict it and you will not
        be able to get any images due to return value None from stream.

        - `fps`: One of 5, 10, 15 or 30 FPS
        - `camera`: One of camera_top, camera_bottom
        - `resolution`:
            - 0: 160x120
            - 1: 320x240
            - 2: 640x480
            - 3: 1280x960
        """
        camera_index = None
        if camera == "camera_top":
            camera_index = 0
        elif camera == "camera_bottom":
            camera_index = 1

        self.camera_link = self.camera_device.subscribeCamera("Camera_Stream" + str(numpy.random),
                                                              camera_index, resolution, 13, fps)
        if self.camera_link:
            print("[INFO]: Camera is initialized")
        else:
            print("[ERROR]: Camera is not initialized properly")

    def unsubscribe_camera(self):
        """Unsubscribe to camera after you don't need it"""
        self.camera_device.unsubscribe(self.camera_link)
        print("[INFO]: Camera was unsubscribed")

    def get_camera_frame(self, show):
        """
        Get camera frame from subscribed camera link.

        Please subscribe to camera before getting a camera frame. After
        you don't need it unsubscribe it.

        - `show`: Show image with OpenCV

        Returns a camera frame
        """
        image_raw = self.camera_device.getImageRemote(self.camera_link)
        image = numpy.frombuffer(image_raw[6], numpy.uint8).reshape(image_raw[1], image_raw[0], 3)

        if show:
            cv2.imshow("Pepper Camera", image)
            cv2.waitKey(-1)
            cv2.destroyAllWindows()

        return image

    def set_security_distance(self, distance=0.05):
        """
        Set security distance. Lower distance for passing doors etc.

        - `distance`: Set distance to the closer objects (default 0.05)
        """
        self.motion_service.setOrthogonalSecurityDistance(distance)
        print("[INFO]: Security distance set to " + str(distance) + " m")

    def move_head_down(self):
        """Look down"""
        self.motion_service.setAngles("HeadPitch", 0.4, 0.2)

    def move_head_up(self):
        """Look up"""
        self.motion_service.setAngles("HeadPitch", -0.4, 0.2)

    def move_head_default(self):
        """Put head into default position in 'StandInit' pose"""
        self.motion_service.setAngles("HeadPitch", 0.0, 0.2)

    def move_to_circle(self, clockwise, t=10):
        """
        Move a robot into circle for specified time

        - `clockwise`: Specifies a direction to turn around
        - `t`: Time in seconds (default 10)
        """
        if clockwise:
            self.motion_service.moveToward(0.5, 0.0, 0.6)
        else:
            self.motion_service.moveToward(0.5, 0.0, -0.6)
        time.sleep(t)
        self.motion_service.stopMove()

    def blink_eyes(self, rgb):
        """
        Blink eyes with defined color

        - `rgb`: Color in RGB space
        """
        self.led_service.fadeRGB('AllLeds', rgb[0], rgb[1], rgb[2], 1.0)

    def navigate_to(self, x, y):
        """
        Navigate robot in map based on exploration mode
        or load previsouly mapped enviroment. Before navigation
        you have to run localization of the robot.

        - `x`: X axis in meters
        - `y`: Y axis in meters
        """
        print("[INFO]: Trying to navigate into specified location")
        try:
            self.navigation_service.startLocalization()
            self.navigation_service.navigateToInMap([x, y, 0])
            self.navigation_service.stopLocalization()
            print("[INFO]: Successfully got into location")
            self.say("At your command")
        except:
            print("[ERROR]: Failed to got into location")
            self.say("I cannot move in that direction")

    def unsubscribe_effector(self):
        """Unsubscribe a end-effector after tracking some object"""
        self.tracker_service.unregisterAllTargets()
        self.tracker_service.setEffector("None")
        print("[INFO]: End-effector is unsubscribed")

    def pick_a_volunteer(self):
        """
        Complex movement for choosing a random people. It robot does not
        see any person it will automatically after several seconds
        turning in one direction and looking for a human. When it detects
        a face it will says 'I found a volunteer' and raise a hand toward
        her/him and move forward. Then it get's into a default 'StandInit'
        pose.
        """
        volunteer_found = False
        self.unsubscribe_effector()
        self.stand()
        self.say("I need a volunteer.")

        proxy_name = "FaceDetection" + str(numpy.random)

        print("[INFO]: Pick a volunteer mode started")

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

    @staticmethod
    def share_localhost(folder):
        """
        Shares a location on localhost via HTTPS to Pepper be
        able to reach it by subscribing to IP address of this
        computer.

        - `folder`: Root folder to share
        """
        # TODO: Add some elegant method to kill a port if previously opened
        subprocess.Popen(["cd", folder])
        subprocess.Popen(["python", "-m", "SimpleHTTPServer"])
        print("[INFO]: HTTPS server successfully started")

    def play_sound(self, sound):
        """
        Play a *.mp3 or *.wav sound stored on Pepper

        - `sound`: Absolute path to the sound
        """
        print("[INFO]: Playing " + sound)
        self.audio_service.playFile(sound)

    def stop_sound(self):
        """Stop sound"""
        print("[INFO]: Stop playing the sound")
        self.audio_service.stopAll()


class VirtualPepper:
    """Virtual robot for testing"""

    def __init__(self):
        """Constructor of virtual robot"""
        print("[INFO]: Using virtual robot!")

    @staticmethod
    def say(text):
        """
        Say some text trough text to speech

        - `text`: Text to speech
        """
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
