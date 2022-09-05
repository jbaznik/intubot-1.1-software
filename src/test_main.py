"""
Main runs
"""
import os
import time
import cv2
# Classes
from computer_vision import ComputerVision
# from driver import Driver
# from serial_handler import SerialHandler
from video_handler import VideoHandler

# TODO: verify that stream 1 is the endoscope camera, stream 2 is the laryngoscope camera.
# VIDEO_STREAM_1 = 'C:/Users/lep/ /01-S/manikin/20200831-hans3/hans3.mp4'  # add video if camera should not be used
VIDEO_STREAM_1 = 0 #r'C:/Users/was/SynologyDrive/01-data/intuBot/1/1.mp4'

VIDEO_STREAM_2 = 1
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

        # setup threads and processes
        self.videohandler = VideoHandler(
            video_stream=self.video_stream_1, frame_w=FRAME_W,
            frame_h=FRAME_H, start_time=self.start_time, recording=self.recording, recording_path=PATH_TO_RECORDINGS)

        
        # self.serialhandler = SerialHandler(self.start_time, recording=self.recording, recording_path=PATH_TO_RECORDINGS)
        self.computervision = ComputerVision()
        # self.driver = Driver(PATH_TO_CONTROL_VALUES, FRAME_W, FRAME_H)
        print("start...")
        self.videohandler.start()  # continuously captures frames
        # self.serialhandler.start()  # continuously read serial inputs
        print("start is done")
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
        while True:
            print("use frame ", self.videohandler.frame_number)

            if not self.videohandler.frame_ready:  # there is no new frame -> wait or stop driving
                print("frame ", self.videohandler.frame_number, "is not ready")
                if not self.videohandler.timestamp_lastframe or time.time() - self.videohandler.timestamp_lastframe < TIMEOUT_TIME:
                    timestamp_lastframe = self.videohandler.timestamp_lastframe
                    print("timestamp_lastframe: ", timestamp_lastframe)
                    print("type: ", type(timestamp_lastframe))
                    print("type: ", type(time.time()))
                    now = time.time()
                    # diff = now - timestamp_lastframe 
                    # print("diff ", diff)
                   
                    print("wait 0.01 second for frame ", self.videohandler.frame_number)
                    # TODO: why sleep here?
                    time.sleep(0.01)  # wait till a frame is ready or time out
                    continue
                # else:
                #     self.driver.emergency_stop()
            else:  # new frame is ready
                print("frame: ", self.videohandler.frame_number, "is ready")
                # self.serialhandler.frame_number = self.videohandler.frame_number

                # freeze passed frame
                self.frame = self.videohandler.frame
                self.videohandler.frame_ready = False # why = False?
         
                # perform detection
                self.scene = self.computervision.object_detection(self.frame)
                self.new_scene = True  # flag for GUI

                # loop through scene to know what has been detected
                #self.driver.track_anatomy_detection(self.scene)

                


    def close_event(self):
        self.videohandler.close()
        self.running = False


if __name__ == '__main__':
    app = MainApp()
    try:
        app.run()
    except KeyboardInterrupt:
        app.close_event()
