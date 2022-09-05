"""
Main runs
"""
import os
import time
# Classes
from computer_vision import ComputerVision
from driver import Driver
from serial_handler import SerialHandler
from video_handler import VideoHandler
from xBox_handler import xBoxHandler


# TODO: verify that stream 1 is the endoscope camera, stream 2 is the laryngoscope camera.
VIDEO_STREAM_1 = 1#'C:/Users/lep/SynologyDrive/01-data/manikin/20200831-hans3/hans3.mp4'  # add video if camera should not be used
VIDEO_STREAM_2 = 0
FRAME_W = 640  # the smaller the frame, the faster the computer vision
FRAME_H = 360
TIMEOUT_TIME = 2  # [seconds], if no image is received within this time, stop the robot

PATH_TO_CODE = os.path.dirname(os.path.realpath(__file__))
PATH_TO_MODEL_DIR = os.path.join(PATH_TO_CODE, '..', 'load')

MODEL_MANIKIN = {
    'name': 'aiE model set',
    'path_to_weights': os.path.join(PATH_TO_MODEL_DIR, "model_manikin", "weights.pth"),
    'path_to_labels': os.path.join(PATH_TO_MODEL_DIR, "model_manikin", "label_list.json"),
    'path_to_thresholds': os.path.join(PATH_TO_MODEL_DIR, "model_manikin", "threshold_map.json")
}

MODEL_COCO = {
    'name': 'common objects',
    'path_to_weights': os.path.join(PATH_TO_MODEL_DIR, "model_coco", "weights.pth"),
    'path_to_labels': os.path.join(PATH_TO_MODEL_DIR, "model_coco", "label_list.json"),
    'path_to_thresholds': os.path.join(PATH_TO_MODEL_DIR, "model_coco", "threshold_map.json")
}

PATH_TO_CONTROL_VALUES = os.path.join(PATH_TO_MODEL_DIR, "control_map.json")

# TODO: choose recording path. Would be nice if it saved directly to a cloud directory.
PATH_TO_RECORDINGS = os.path.join(PATH_TO_CODE, '..', 'log')


class MainApp():

    def __init__(self):
        self.start_time = time.time()
        self.button_press_time = 0
        self.running = True
        self.recording = False
        self.auto_mode = False
        self.video_stream_1 = VIDEO_STREAM_1
        self.video_stream_2 = VIDEO_STREAM_2
        self.model_manikin = MODEL_MANIKIN
        self.model_coco = MODEL_COCO
        self.new_scene = False
        self.scene = {}
        self.frame = None
        self.system_hardware_brought_to_home_position = False
        self.go_home = False
        self.track_cup = False
        # setup threads and processes
        self.videohandler = VideoHandler(
            video_stream=self.video_stream_1, frame_w=FRAME_W,
            frame_h=FRAME_H, start_time=self.start_time, recording=self.recording, recording_path=PATH_TO_RECORDINGS)
        self.serialhandler = SerialHandler(self.start_time, recording=self.recording, recording_path=PATH_TO_RECORDINGS)
        self.computervision = ComputerVision()
        self.driver = Driver(PATH_TO_CONTROL_VALUES, FRAME_W, FRAME_H)
        self.xboxhandler = xBoxHandler()

        self.videohandler.start()  # continuously captures frames
        self.serialhandler.start()  # continuously read serial inputs
        self.xboxhandler.start() # continuously ready xBox inputs
        
        

        self.computervision.load(
            self.model_manikin['name'],
            self.model_manikin['path_to_weights'],
            self.model_manikin['path_to_labels'],
            self.model_manikin['path_to_thresholds'])  # initiates PyTorch

        # set up secondary camera
        # secondary camera is only for display, it won't be used by computer vision and visual servioing.
        if self.video_stream_2 is not None and self.video_stream_2 != self.video_stream_1:
            self.videohandler_secondary = VideoHandler(
                video_stream=self.video_stream_2, frame_w=FRAME_W, frame_h=FRAME_H, start_time=self.start_time,
                recording=self.recording, recording_path=PATH_TO_RECORDINGS)
            self.videohandler_secondary.start()
        else:
            self.videohandler_secondary = None



    def run(self):
        """
        waits till a frame is ready.
        once a frame is ready, performs object detection on it, calculates motor controls and drives
        """
        """
        # bring to home position
        if not self.system_hardware_brought_to_home_position:
            # first correct endoscope position

            # bring
        """

        while True:
            if not self.videohandler.frame_ready:  # there is no new frame -> wait or stop driving
                if not self.videohandler.timestamp_lastframe or time.time() - self.videohandler.timestamp_lastframe < TIMEOUT_TIME:
                    # TODO: why sleep here?
                    time.sleep(0.01)  # wait till a frame is ready or time out
                    continue
                else:
                    self.driver.emergency_stop()
            else:  # new frame is ready
                self.serialhandler.frame_number = self.videohandler.frame_number

                # freeze passed frame
                self.frame = self.videohandler.frame
                self.videohandler.frame_ready = False

                # perform detection
                self.scene = self.computervision.object_detection(self.frame)
                
                # detect only cups when using the coco model
                if self.computervision.model_name == self.model_coco['name']:
                    if 'cup' in self.scene:
                        self.scene = {'cup': self.scene['cup']}
                    if 'cup' not in self.scene:
                        self.scene = {}
                self.new_scene = True  # flag for GUI

                if not self.computervision.common_objects_running:
                    # loop through scene to know what has been detected
                    self.driver.track_anatomy_detection(self.scene)
                else:
                    self.driver.track_common_object(self.scene)

                # update button states
                # add logic here if auto mode should stay active when button is released
                if self.serialhandler.button_auto:
                    self.auto_mode = True
                else:
                    self.auto_mode = False
            
#                if self.go_home:
 #                   self.driver.converge_to_home_position()

                # run control function
                self.xboxhandler.read()
                self.button_press_time 
                if not self.computervision.common_objects_running:
                    if (self.auto_mode or self.xboxhandler.xBox_inputs['button_B']) and self.driver.auto_mode_available:
                        self.driver.visual_servoing(self.scene, 
                                                self.serialhandler.joystick,
                                                self.serialhandler.analog_rocker, 
                                                self.serialhandler.motor)
                    else:
                        self.driver.manual_driving(self.serialhandler.joystick, 
                                            self.xboxhandler.xBox_connected,
                                            self.xboxhandler.xBox_inputs, 
                                            self.serialhandler.analog_rocker, 
                                            self.serialhandler.motor)

                elif self.computervision.common_objects_running:
                    if (self.auto_mode or self.xboxhandler.xBox_inputs['button_B']) and not self.track_cup and ((time.time() - self.button_press_time) > 1 ):
                        self.track_cup = True
                        self.button_press_time = time.time()
                    elif (self.auto_mode or self.xboxhandler.xBox_inputs['button_B']) and self.track_cup and ((time.time() - self.button_press_time) > 1 ):
                        self.track_cup = False
                        self.button_press_time = time.time()
                    
                    if self.track_cup and self.driver.auto_mode_available:
                        self.driver.visual_servoing_on_object(self.scene, self.serialhandler.joystick,
                                                self.serialhandler.analog_rocker, self.serialhandler.motor)
                    else:
                        self.driver.manual_driving(self.serialhandler.joystick, 
                                            self.xboxhandler.xBox_connected,
                                            self.xboxhandler.xBox_inputs, 
                                            self.serialhandler.analog_rocker, 
                                            self.serialhandler.motor)
                 

            # send command
            # if self.driver.auto_mode_available:
            #     led_auto = 255
            # else:
            #     led_auto = 0

            if len(self.scene) != 0:
                led_auto = 255
            
            else:
                led_auto = 0

            print(self.driver.motor_pwm_command)
            # prevent sled from driving past the magnet
            self.driver.sled_boundary_control(self.serialhandler.hall_sensor)

            self.serialhandler.send(motors=self.driver.motor_pwm_command,
                                    led_auto= led_auto, # TODO Reset to led_auto,
                                    led_display = 128,
                                    led_laryngoscope=255,
                                    stop=self.driver.stop)


            # self.serialhandler.joystick

            

            


    def close_event(self):
        self.videohandler.close()
        self.xboxhandler.close()
        self.serialhandler.close()
        self.running = False


if __name__ == '__main__':
    app = MainApp()
    try:
        app.run()
    except KeyboardInterrupt:
        app.close_event()
