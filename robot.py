"""
This is a wrapper around `qi` framework by Aldebaran
to control Pepper the humanoid robot with Python 2.7.

Package uses high-level commands to move robot, take
camera input or run Google Recognition API to get speech
recognition.

It also includes a virtual robot for testing purposes.
"""
import qi
import time
import numpy
import cv2
import speech_recognition
import gtts
import playsound
import subprocess
import dance
import socket
import paramiko
from scp import SCPClient
import tools


class Pepper:
    """
    **Real robot controller**

    Create an instance of real robot controller by specifying
    a robot's IP address and port. IP address can be:

    - hostname (hostname like `pepper.local`)
    - IP address (can be obtained by pressing robot's *chest button*)

    Default port is usually used, e.g. `9559`.

    :Example:

    >>> pepper = Pepper("pepper.local")
    >>> pepper = Pepper("192.169.0.1", 1234)

    """
    def __init__(self, ip_address, port=9559):
        self.session = qi.Session()
        self.session.connect("tcp://" + ip_address + ":" + str(port))

        self.ip_address = ip_address
        self.port = port

        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.load_system_host_keys()
        ssh.connect(hostname=self.ip_address, username="nao", password="nao")
        self.scp = SCPClient(ssh.get_transport())

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
        self.audio_service = self.session.service("ALAudioPlayer")
        self.animation_service = self.session.service("ALAnimationPlayer")
        self.behavior_service = self.session.service("ALBehaviorManager")
        self.face_characteristic = self.session.service("ALFaceCharacteristics")
        self.people_perception = self.session.service("ALPeoplePerception")
        self.speech_service = self.session.service("ALSpeechRecognition")
        self.dialog_service = self.session.service("ALDialog")
        self.audio_recorder = self.session.service("ALAudioRecorder")

        self.slam_map = None
        self.localization = None
        self.camera_link = None

        self.recognizer = speech_recognition.Recognizer()

        print("[INFO]: Robot is initialized at " + ip_address + ":" + str(port))

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
        Point end-effector in cartesian space

        :Example:

        >>> pepper.point_at(1.0, 1.0, 0.0, "RArm", 0)

        :param x: X axis in meters
        :type x: float
        :param y: Y axis in meters
        :type y: float
        :param z: Z axis in meters
        :type z: float
        :param effector_name: `LArm`, `RArm` or `Arms`
        :type effector_name: string
        :param frame: 0 = Torso, 1 = World, 2 = Robot
        :type frame: integer
        """
        speed = 0.5     # 50 % of speed
        self.tracker_service.pointAt(effector_name, [x, y, z], frame, speed)

    def move_forward(self, speed):
        """
        Move forward with certain speed

        :param speed: Positive values forward, negative backwards
        :type speed: float
        """
        self.motion_service.move(speed, 0, 0)

    def turn_around(self, speed):
        """
        Turn around its axis

        :param speed: Positive values to right, negative to left
        :type speed: float
        """
        self.motion_service.move(0, 0, speed)

    def stop_moving(self):
        """Stop robot from moving by `move_around` and `turn_around` methods"""
        self.motion_service.stopMove()

    def say(self, text):
        """
        Text to speech (robot internal engine)

        :param text: Text to speech
        :type text: string
        """
        self.tts_service.say(text)
        print("[INFO]: Robot says: " + text)

    def tablet_show_web(self, url):
        """
        Display a web page on robot's tablet. It also works for
        sharing a locally stored images and websites by running:

        >>> pepper.share_localhost("/Users/user/Desktop/web_host/")
        >>> pepper.tablet_show_web("<remote_ip>:8000/web_host/index.html")

        Or

        >>> pepper.tablet_show_web("https://www.ciirc.cvut.cz")

        .. note:: Or you can simply run `python -m SimpleHTTPServer` in the root of \
        the web host and host it by self on specified port. Default is 8000.

        :param url: Web URL
        :type  url: string
        """
        self.tablet_service.showWebview(url)

    def clean_tablet(self):
        """Clean tablet and show default tablet animation on robot"""
        self.tablet_service.hideWebview()

    def tablet_show_image(self, image_url):
        """
        Display image on robot tablet

        .. seealso:: For more take a look at `tablet_show_web()`

        :Example:

        >>> pepper.tablet_show_image("https://goo.gl/4Xq6Bc")

        :param image_url: Image URL (local or web)
        :type image_url: string
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
        """
        Switch autonomous life off

        .. note:: After switching off, robot stays in resting posture. After \
        turning autonomous life default posture is invoked
        """
        self.autonomous_life_service.setState("disabled")
        self.stand()
        print("[INFO]: Autonomous life is off")

    def autonomous_life_on(self):
        """Switch autonomous life on"""
        self.autonomous_life_service.setState("interactive")
        print("[INFO]: Autonomous life is on")

    def track_object(self, object_name, effector_name, diameter=0.05):
        """
        Track a object with a given object type and diameter. If `Face` is
        chosen it has a default parameters to 15 cm diameter per face. After
        staring tracking in will wait until user press ctrl+c.

        .. seealso:: For more info about tracking modes, object names and other:\
        http://doc.aldebaran.com/2-5/naoqi/trackers/index.html#tracking-modes

        :Example:

        >>> pepper.track_object("Face", "Arms")

        Or

        >>> pepper.track_object("RedBall", "LArm", diameter=0.1)

        :param object_name: `RedBall`, `Face`, `LandMark`, `LandMarks`, `People` or `Sound`
        :param effector_name: `LArm`, `RArm`, `Arms`
        :param diameter: Diameter of the object (default 0.05, for face default 0.15)
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

        .. seealso:: When robot would not move maybe it only needs \
        to set smaller safety margins. Take a look and `set_security_distance()`

        .. note:: Default folder for saving maps on the robot is: \
        `/home/nao/.local/share/Explorer/`

        :param radius: Distance in meters
        :type radius: integer
        :return: image
        :rtype: cv2 image
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

        :param on_robot: If set shows a map on the robot
        :type on_robot: bool
        :param remote_ip: IP address of remote (default None)
        :type remote_ip: string

        .. warning:: Showing map on the robot is not fully supported at the moment.
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
        """
        Localize a robot in a map

        .. note:: After loading a map into robot or after new exploration \
        robots always need to run a self localization. Even some movement in \
        cartesian space demands localization.
        """
        # TODO: There should be localizeInMap() with proper coordinates
        try:
            self.navigation_service.startLocalization()
            localization = self.navigation_service.getRobotPositionInMap()
            self.localization = localization[0]
            print("[INFO]: Localization complete")
            self.navigation_service.stopLocalization()
        except Exception as error:
            print(error)
            print("[ERROR]: Localization failed")

    def stop_localization(self):
        """Stop localization of the robot"""
        self.navigation_service.stopLocalization()
        print("[INFO]: Localization stopped")

    def load_map(self, file_name, file_path="/home/nao/.local/share/Explorer/"):
        """
        Load stored map on a robot. It will find a map in default location,
        in other cases alternative path can be specifies by `file_name`.

        .. note:: Default path of stored maps is `/home/nao/.local/share/Explorer/`

        .. warning:: If file path is specified it is needed to have `\` at the end.

        :param file_name: Name of the map
        :type file_name: string
        :param file_path: Path to the map
        :type file_path: string
        """
        self.slam_map = self.navigation_service.loadExploration(file_path + file_name)
        print("[INFO]: Map '" + file_name + "' loaded")

    def set_volume(self, volume):
        """
        Set robot volume in percentage

        :param volume: From 0 to 100 %
        :type volume: integer
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

        :param state: If True set on, if False set off
        :type state: bool
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
        before you reach a images from it. If you choose `depth_camera`
        only 320x240 resolution is enabled.

        .. warning:: Each subscription has to have a unique name \
        otherwise it will conflict it and you will not be able to \
        get any images due to return value None from stream.

        :Example:

        >>> pepper.subscribe_camera(0, 1, 15)
        >>> image = pepper.get_camera_frame(False)
        >>> pepper.unsubscribe_camera()

        :param camera: `camera_depth`, `camera_top` or `camera_bottom`
        :type camera: string
        :param resolution:
            0. 160x120
            1. 320x240
            2. 640x480
            3. 1280x960
        :type resolution: integer
        :param fps: Frames per sec (5, 10, 15 or 30)
        :type fps: integer
        """
        color_space = 13

        camera_index = None
        if camera == "camera_top":
            camera_index = 0
        elif camera == "camera_bottom":
            camera_index = 1
        elif camera == "camera_depth":
            camera_index = 2
            resolution = 1
            color_space = 11

        self.camera_link = self.camera_device.subscribeCamera("Camera_Stream" + str(numpy.random.random()),
                                                              camera_index, resolution, color_space, fps)
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

        .. warning:: Please subscribe to camera before getting a camera frame. After \
        you don't need it unsubscribe it.

        :param show: Show image when recieved and wait for `ESC`
        :type show: bool
        :return: image
        :rtype: cv2 image
        """
        image_raw = self.camera_device.getImageRemote(self.camera_link)
        image = numpy.frombuffer(image_raw[6], numpy.uint8).reshape(image_raw[1], image_raw[0], 3)

        if show:
            cv2.imshow("Pepper Camera", image)
            cv2.waitKey(-1)
            cv2.destroyAllWindows()

        return image

    def get_depth_frame(self, show):
        """
        Get depth frame from subscribed camera link.

        .. warning:: Please subscribe to camera before getting a camera frame. After \
        you don't need it unsubscribe it.

        :param show: Show image when recieved and wait for `ESC`
        :type show: bool
        :return: image
        :rtype: cv2 image
        """
        image_raw = self.camera_device.getImageRemote(self.camera_link)
        image = numpy.frombuffer(image_raw[6], numpy.uint8).reshape(image_raw[1], image_raw[0], 3)

        if show:
            cv2.imshow("Pepper Camera", image)
            cv2.waitKey(-1)
            cv2.destroyAllWindows()

        return image

    def show_tablet_camera(self, text):
        """
        Show image from camera with SpeechToText annotation on the robot tablet

        .. note:: For showing image on robot you will need to share a location via HTTPS and \
        save the image to ./tmp.

        .. warning:: It has to be some camera subscribed and ./tmp folder in root directory \
        exists for showing it on the robot.

        :Example:

        >>> pepper = Pepper("10.37.1.227")
        >>> pepper.share_localhost("/Users/michael/Desktop/Pepper/tmp/")
        >>> pepper.subscribe_camera("camera_top", 2, 30)
        >>> while True:
        >>>     pepper.show_tablet_camera("camera top")
        >>>     pepper.tablet_show_web("http://10.37.2.241:8000/tmp/camera.png")

        :param text: Question of the visual question answering
        :type text: string
        """
        remote_ip = socket.gethostbyname(socket.gethostname())
        image_raw = self.camera_device.getImageRemote(self.camera_link)
        image = numpy.frombuffer(image_raw[6], numpy.uint8).reshape(image_raw[1], image_raw[0], 3)
        image = cv2.resize(image, (800, 600))
        cv2.putText(image, "Visual question answering", (30, 500), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        cv2.putText(image, "Question: " + text, (30, 550), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.imwrite("./tmp/camera.png", image)

        self.tablet_show_web("http://" + remote_ip + ":8000/tmp/camera.png")

    def set_security_distance(self, distance=0.05):
        """
        Set security distance. Lower distance for passing doors etc.

        .. warning:: It is not wise to turn `security distance` off.\
        Robot may fall from stairs or bump into any fragile objects.

        :Example:

        >>> pepper.set_security_distance(0.01)

        :param distance: Distance from the objects in meters
        :type distance: float
        """
        self.motion_service.setOrthogonalSecurityDistance(distance)
        print("[INFO]: Security distance set to " + str(distance) + " m")

    def move_head_down(self):
        """Look down"""
        self.motion_service.setAngles("HeadPitch", 0.46, 0.2)

    def move_head_up(self):
        """Look up"""
        self.motion_service.setAngles("HeadPitch", -0.4, 0.2)

    def move_head_default(self):
        """Put head into default position in 'StandInit' pose"""
        self.motion_service.setAngles("HeadPitch", 0.0, 0.2)

    def move_to_circle(self, clockwise, t=10):
        """
        Move a robot into circle for specified time

        .. note:: This example only count on time not finished circles.

        >>> pepper.move_to_circle(clockwise=True, t=5)

        :param clockwise: Specifies a direction to turn around
        :type clockwise: bool
        :param t: Time in seconds (default 10)
        :type t: float
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

        :param rgb: Color in RGB space
        :type rgb: integer

        :Example:

        >>> pepper.blink_eyes([255, 0, 0])

        """
        self.led_service.fadeRGB('AllLeds', rgb[0], rgb[1], rgb[2], 1.0)

    def turn_off_leds(self):
        """Turn off the LEDs in robot's eyes"""
        self.blink_eyes([0, 0, 0])

    def navigate_to(self, x, y):
        """
        Navigate robot in map based on exploration mode
        or load previously mapped enviroment.

        .. note:: Before navigation you have to run localization of the robot.

        .. warning:: Navigation to 2D point work only up to 3 meters from robot.

        :Example:

        >>> pepper.robot_localization()
        >>> pepper.navigate_to(1.0, 0.3)

        :param x: X axis in meters
        :type x: float
        :param y: Y axis in meters
        :type y: float
        """
        print("[INFO]: Trying to navigate into specified location")
        try:
            self.navigation_service.startLocalization()
            self.navigation_service.navigateToInMap([x, y, 0])
            self.navigation_service.stopLocalization()
            print("[INFO]: Successfully got into location")
            self.say("At your command")
        except Exception as error:
            print(error)
            print("[ERROR]: Failed to got into location")
            self.say("I cannot move in that direction")

    def unsubscribe_effector(self):
        """
        Unsubscribe a end-effector after tracking some object

        .. note:: It has to be done right after each tracking by hands.
        """
        self.tracker_service.unregisterAllTargets()
        self.tracker_service.setEffector("None")
        print("[INFO]: End-effector is unsubscribed")

    def pick_a_volunteer(self):
        """
        Complex movement for choosing a random people.

        If robot does not see any person it will automatically after several
        seconds turning in one direction and looking for a human. When it detects
        a face it will says 'I found a volunteer' and raise a hand toward
        her/him and move forward. Then it get's into a default 'StandInit'
        pose.

        :Example:

        >>> pepper.pick_a_volunteer()

        """
        volunteer_found = False
        self.unsubscribe_effector()
        self.stand()
        self.say("I need a volunteer.")

        proxy_name = "FaceDetection" + str(numpy.random)

        print("[INFO]: Pick a volunteer mode started")

        while not volunteer_found:
            wait = numpy.random.randint(500, 1500) / 1000
            theta = numpy.random.randint(-10, 10)
            self.turn_around(theta)
            time.sleep(wait)
            self.stop_moving()
            self.stand()
            self.face_detection_service.subscribe(proxy_name, 500, 0.0)
            for memory in range(5):
                time.sleep(0.5)
                output = self.memory_service.getData("FaceDetected")
                print("...")
                if output and isinstance(output, list) and len(output) >= 2:
                    print("Face detected")
                    volunteer_found = True

        self.say("I found a volunteer! It is you!")
        self.stand()
        try:
            self.tracker_service.registerTarget("Face", 0.15)
            self.tracker_service.setMode("Move")
            self.tracker_service.track("Face")
            self.tracker_service.setEffector("RArm")
            self.get_face_properties()

        finally:
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

        :Example:

        >>> pepper.share_localhost("/Users/pepper/Desktop/web/")

        :param folder: Root folder to share
        :type folder: string
        """
        # TODO: Add some elegant method to kill a port if previously opened
        subprocess.Popen(["cd", folder])
        try:
            subprocess.Popen(["python", "-m", "SimpleHTTPServer"])
        except Exception as error:
            subprocess.Popen(["python", "-m", "SimpleHTTPServer"])
        print("[INFO]: HTTPS server successfully started")

    def play_sound(self, sound):
        """
        Play a `mp3` or `wav` sound stored on Pepper

        .. note:: This is working only for songs stored in robot.

        :param sound: Absolute path to the sound
        :type sound: string
        """
        print("[INFO]: Playing " + sound)
        self.audio_service.playFile(sound)

    def stop_sound(self):
        """Stop sound"""
        print("[INFO]: Stop playing the sound")
        self.audio_service.stopAll()

    def start_animation(self, animation):
        """
        Starts a animation which is stored on robot

        .. seealso:: Take a look a the animation names in the robot \
        http://doc.aldebaran.com/2-5/naoqi/motion/alanimationplayer.html#alanimationplayer

        :param animation: Animation name
        :type animation: string
        :return: True when animation has finished
        :rtype: bool
        """
        try:
            animation_finished = self.animation_service.run("animations/[posture]/Gestures/" + animation, _async=True)
            animation_finished.value()
            return True
        except Exception as error:
            print(error)
            return False

    def start_dance(self):
        """
        Start a robotic dance
        """
        dance.dance(self.motion_service)

    def start_behavior(self, behavior):
        """
        Starts a behavior stored on robot

        :param behavior: Behavior name
        :type behavior: string
        """
        self.behavior_service.startBehavior(behavior)

    def list_behavior(self):
        """Prints all installed behaviors on the robot"""
        print(self.behavior_service.getBehaviorNames())

    def get_face_properties(self):
        """
        Gets all face properties from the tracked face in front of
        the robot.

        It tracks:
        - Emotions (neutral, happy, surprised, angry and sad
        - Age
        - Gender

        .. note:: It also have a feature that it substracts a 5 year if it talks to a female.

        .. note:: If it cannot decide which gender the user is, it just greets her/him as "Hello human being"

        ..warning:: To get this feature working `ALAutonomousLife` process is needed. In this methods it is \
        called by default
        """
        self.autonomous_life_on()
        emotions = ["neutral", "happy", "surprised", "angry", "sad"]
        face_id = self.memory_service.getData("PeoplePerception/PeopleList")
        recognized = None
        try:
            recognized = self.face_characteristic.analyzeFaceCharacteristics(face_id[0])
        except Exception as error:
            print("[ERROR]: Cannot find a face to analyze.")
            self.say("I cannot recognize a face.")

        if recognized:
            properties = self.memory_service.getData("PeoplePerception/Person/" + str(face_id[0]) + "/ExpressionProperties")
            gender = self.memory_service.getData("PeoplePerception/Person/" + str(face_id[0]) + "/GenderProperties")
            age = self.memory_service.getData("PeoplePerception/Person/" + str(face_id[0]) + "/AgeProperties")

            # Gender properties
            if gender[1] > 0.4:
                if gender[0] == 0:
                    self.say("Hello lady!")
                elif gender[0] == 1:
                    self.say("Hello sir!")
            else:
                self.say("Hello human being!")

            # Age properties
            if gender[1] == 1:
                self.say("You are " + str(int(age[0])) + " years old.")
            else:
                self.say("You look like " + str(int(age[0])) + " oops, I mean " + str(int(age[0]-5)))

            # Emotion properties
            emotion_index = (properties.index(max(properties)))

            if emotion_index > 0.5:
                self.say("I am quite sure your mood is " + emotions[emotion_index])
            else:
                self.say("I guess your mood is " + emotions[emotion_index])

    def listen_to(self, vocabulary):
        """
        Listen and match the vocabulary which is passed as parameter.

        :Example:

        >>> words = pepper.listen_to(["what color is the sky", "yes", "no"]

        :param vocabulary: List of phrases or words to recognize
        :type vocabulary: string
        :return: Recognized phrase or words
        :rtype: string
        """
        self.speech_service.setLanguage("English")
        self.speech_service.pause(True)
        try:
            self.speech_service.setVocabulary(vocabulary, True)
        except RuntimeError as error:
            print(error)
            self.speech_service.removeAllContext()
            self.speech_service.setVocabulary(vocabulary, True)
            self.speech_service.subscribe("Test_ASR")
        try:
            print("[INFO]: Robot is listening to you...")
            self.speech_service.pause(False)
            time.sleep(4)
            words = self.memory_service.getData("WordRecognized")
            print("[INFO]: Robot understood: '" + words[0] + "'")
            return words[0]
        except:
            pass

    def listen(self):
        """
        Wildcard speech recognition via internal Pepper engine

        .. warning:: To get this proper working it is needed to disable or uninstall \
        all application which can modify a vocabulary in a Pepper.

        .. note:: Note this version only rely on time but not its internal speak processing \
        this means that Pepper will 'bip' at the begining and the end of human speak \
        but it is not taken a sound in between the beeps. Search for 'Robot is listening to \
        you ... sentence in log console

        :Example:

        >>> words = pepper.listen()

        :return: Speech to text
        :rtype: string
        """
        self.speech_service.setAudioExpression(False)
        self.speech_service.setVisualExpression(False)
        self.audio_recorder.stopMicrophonesRecording()
        print("[INFO]: Speech recognition is in progress. Say something.")
        while True:
            print(self.memory_service.getData("ALSpeechRecognition/Status"))
            if self.memory_service.getData("ALSpeechRecognition/Status") == "SpeechDetected":
                self.audio_recorder.startMicrophonesRecording("/home/nao/speech.wav", "wav", 48000, (0, 0, 1, 0))
                print("[INFO]: Robot is listening to you")
                self.blink_eyes([255, 255, 0])
                break

        while True:
            if self.memory_service.getData("ALSpeechRecognition/Status") == "EndOfProcess":
                self.audio_recorder.stopMicrophonesRecording()
                print("[INFO]: Robot is not listening to you")
                self.blink_eyes([0, 0, 0])
                break

        self.download_file("speech.wav")
        self.speech_service.setAudioExpression(True)
        self.speech_service.setVisualExpression(True)

        return self.speech_to_text("speech.wav")

    def ask_wikipedia(self):
        """
        Ask for question and then robot will say first two sentences from Wikipedia

        ..warning:: Autonomous life has to be turned on to process audio
        """
        self.speech_service.setAudioExpression(False)
        self.speech_service.setVisualExpression(False)
        self.set_awareness(False)
        self.say("Give me a question")
        question = self.listen()
        self.say("I will tell you")
        answer = tools.get_knowledge(question)
        self.say(answer)
        self.set_awareness(True)
        self.speech_service.setAudioExpression(True)
        self.speech_service.setVisualExpression(True)

    def rename_robot(self):
        """Change current name of the robot"""
        choice = raw_input("Are you sure you would like to rename a robot? (yes/no)\n")
        if choice == "yes":
            new_name = raw_input("Enter a new name for the robot. Then it will reboot itself.\nName: ")
            self.system_service.setRobotName(new_name)
            self.restart_robot()

    def upload_file(self, file_name):
        """
        Upload file to the home directory of the robot

        :param file_name: File name with extension (or path)
        :type file_name: string
        """
        self.scp.put(file_name)
        print("[INFO]: File " + file_name + " uploaded")
        self.scp.close()

    def download_file(self, file_name):
        """
        Download a file from robot to ./tmp folder in root.

        ..warning:: Folder ./tmp has to exist!
        :param file_name: File name with extension (or path)
        :type file_name: string
        """
        self.scp.get(file_name, local_path="/tmp/")
        print("[INFO]: File " + file_name + " downloaded")
        self.scp.close()

    def speech_to_text(self, audio_file):
        """
        Translate speech to text via Google Speech API

        :param audio_file: Name of the audio (default `speech.wav`
        :type audio_file: string
        :return: Text of the speech
        :rtype: string
        """
        audio_file = speech_recognition.AudioFile("/tmp/" + audio_file)
        with audio_file as source:
            audio = self.recognizer.record(source)
            recognized = self.recognizer.recognize_google(audio, language="en_US")
        return recognized

    def get_robot_name(self):
        """
        Gets a current name of the robot

        :return: Name of the robot
        :rtype: string
        """
        name = self.system_service.robotName()
        if name:
            self.say("My name is " + name)
        return name

    def hand(self, hand, close):
        """
        Close or open hand

        :param hand: Which hand
            - left
            - right
        :type hand: string
        :param close: True if close, false if open
        :type close: boolean
        """
        hand_id = None
        if hand == "left":
            hand_id = "LHand"
        elif hand == "right":
            hand_id = "RHand"

        if hand_id:
            if close:
                self.motion_service.setAngles(hand_id, 0.0, 0.2)
                print("[INFO]: Hand " + hand + "is closed")
            else:
                self.motion_service.setAngles(hand_id, 1.0, 0.2)
                print("[INFO]: Hand " + hand + "is opened")
        else:
            print("[INFO]: Cannot move a hand")

    def chatbot(self):
        """
        Run chatbot with text to speech and speech to text

        ..warning:: This is not currently working
        """
        tools.chatbot_init()
        while True:
            try:
                self.set_awareness(False)
                question = self.listen()
                print("[USER]: " + question)
                answer = tools.chatbot_ask(question)
                print("[ROBOT]: "+ answer)
                self.say(answer)
            except KeyboardInterrupt:
                self.set_awareness(True)


class VirtualPepper:
    """Virtual robot for testing"""

    def __init__(self):
        """Constructor of virtual robot"""
        print("[INFO]: Using virtual robot!")

    @staticmethod
    def say(text):
        """
        Say some text trough text to speech

        :param text: Text to speech
        :type text: string
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
