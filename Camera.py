import cv2
from threading import Thread
from calibrate import Calibrate

class Camera:

    def __init__(self):
        self.video_capture = cv2.VideoCapture(0)
        self.set(3, 1280)
        self.set(4, 720)
        self.current_frame = self.video_capture.read()[1]

    def start(self):
        Thread(target=self._update_frame, args=()).start()

    def _update_frame(self):
        while(True):
            self.current_frame = self.video_capture.read()[1]

    def get_current_frame(self):
        return self.current_frame

    def release_camera(self, cap):
        cap.release()
        cv2.destroyAllWindows()

    def calibrate(self):
        object_3d.cal.calibrate_camera()

        self.cal = Calibrate(path, search_aruco_dict)