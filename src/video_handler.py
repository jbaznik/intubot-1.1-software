import os
import threading
import time
import cv2


class VideoHandler():
    def __init__(self, video_stream=0, frame_w=0, frame_h=0, start_time=None, recording=False, recording_path=None):
        self.video_stream = video_stream
        self.cap = None

        self.running = True
        self.thread = None

        self.recording = recording
        self.recording_path = recording_path
        self.start_time = start_time

        self.frame_w = frame_w
        self.frame_h = frame_h

        self.frame = None
        self.frame_ready = False
        self.frame_number = 0
        self.timestamp_lastframe = None
        self.frame_rate = None

        self.initiating = True

    def init_capture(self):
        """
        initiates video capture
        if self.video_stream is a string, it is interpreted as a path to a video file
        otherise, it looks for a camera with name self.video_stream
        """
        self.frame = None

        if isinstance(self.video_stream, str):  # it's a video file
            self.cap = cv2.VideoCapture(self.video_stream)
            self.frame_rate = self.cap.get(cv2.CAP_PROP_FPS)
        else:  # it's a cam
            self.cap = cv2.VideoCapture(self.video_stream + cv2.CAP_DSHOW)  # DSHOW = direct show device on Windows
        if self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_w)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_h)
            self.frame_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.frame_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

            print(
                f'camera resolution: {self.frame_w} x {self.frame_h}')

            # Finished initiating camera
            self.initiating = False
        else:
            #print('Error: Could not open camera stream or video not found')
            #print('Retry in 1s')
            time.sleep(1)
            self.init_capture()

    def start(self):
        """
        fire up the thread
        """
        self.thread = threading.Thread(target=self.get, args=(), daemon=True)
        self.thread.start()
        self.running = True

    def get(self):
        """
        captures the video frames
        """
        if not self.cap:
            self.init_capture()

        while self.running:
            if self.initiating:
                continue

            (new_get, self.frame) = self.cap.read()
            if new_get:
                self.frame_number = self.frame_number + 1
                self.frame_ready = True
                self.timestamp_lastframe = time.time()
                if self.recording:
                    self.save_frame()
                if self.frame_rate:
                    time.sleep(1./self.frame_rate)  # only if input is a video
            else:  # if cap.read() fails, set frame to None
                self.frame_ready = False
                self.frame = None
                self.init_capture()

    def save_frame(self):
        """
        saves a frame on the harddrive
        """
        if not self.recording_path:
            print('path to recording folder no found -> cannot record session')
            return
        path = os.path.join(self.recording_path, 'frames', f'{self.frame_number}.jpg')
        cv2.imwrite(path, self.frame)

    def close(self):
        """
        closes the video capture
        """
        self.running = False
        self.frame = None
        self.frame_ready = False
        self.thread.join()
