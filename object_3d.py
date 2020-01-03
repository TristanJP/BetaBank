import time
import cv2
from cv2 import aruco
import numpy as np
from glob import glob
from matplotlib import pyplot as plt
from calibrate import Calibrate

class Object_3d:

    CALIBRATION_IMAGE_PATH: str
    cal = None

    def __init__(self, path):
        self.CALIBRATION_IMAGE_PATH = path

        self.cal = Calibrate(self.CALIBRATION_IMAGE_PATH)

if __name__ == "__main__":
    object_3d = Object_3d("calibration_images")
    object_3d.cal.calibrate_camera()

    object_3d.cal.show_pose()