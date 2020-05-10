import cv2
from threading import Thread
from .calibrate import Calibrate

class Camera:

    calibration_data: dict

    def __init__(self, calibration_data = None):
        self.video_capture = cv2.VideoCapture(0)
        self.video_capture.set(3, 1920)
        self.video_capture.set(4, 1080)
        self.video_capture.set(5, 60)

        if calibration_data is not None:
            self.calibration_data = calibration_data
        else:
            self.calibrate()

        read = self.video_capture.read()
        self.current_frame = read[1]
        self.successful_read = read[0]

    def start(self):
        Thread(target=self._update_frame, args=(), daemon = True).start()

    def _update_frame(self):
        while(True):
            read = self.video_capture.read()
            self.current_frame = read[1]
            self.successful_read = read[0]

    def get_current_frame(self):
        return self.current_frame

    def release_camera(self):
        self.video_capture.release()
        cv2.destroyAllWindows()

    def calibrate(self, path="calibration_images_1920x1080"):
        cal = Calibrate(path)
        self.calibration_data = cal.calibrate_camera()

    def get_calibration_data(self):
        return self.calibration_data

if __name__ == "__main__":
    cam = Camera()
    cam.start()

    while (True):
        frame = cam.current_frame
        cv2.imshow("frame", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            cam.release_camera()
            break