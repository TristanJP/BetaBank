import cv2
from threading import Thread
from calibrate import Calibrate

class Camera:

    def __init__(self):
        self.video_capture = cv2.VideoCapture(0)
        self.video_capture.set(3, 1280)
        self.video_capture.set(4, 720)
        self.calibrate()
        self.current_frame = self.video_capture.read()[1]

    def start(self):
        Thread(target=self._update_frame, args=()).start()

    def _update_frame(self):
        while(True):
            self.current_frame = self.video_capture.read()[1]

    def get_current_frame(self):
        return self.current_frame

    def release_camera(self):
        self.video_capture.release()
        cv2.destroyAllWindows()

    def calibrate(self):
        self.cal = Calibrate("calibration_images", cv2.aruco.DICT_6X6_250)
        self.calibration_data = self.cal.calibrate_camera()

    def get_calibration_data(self):
        return self.calibration_data