import cv2
from .calibrate import Calibrate
from .camera import Camera
from .effects import Effects

class View:

    calibration_data: dict

    def __init__(self, calibration_data):
        self.calibration_data = calibration_data

    def render_origin(self, frame, ret, origin_rvec, origin_tvec, shape="cube"):
        effects = Effects()

        if origin_tvec is not None:
            effects.render(frame, self.calibration_data["cam_mtx"], self.calibration_data["dist_coef"], ret, origin_rvec, origin_tvec, shape)
        cv2.imshow("frame", frame)
