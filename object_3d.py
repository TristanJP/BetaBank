import time
import cv2
from cv2 import aruco
import numpy as np
from glob import glob
from matplotlib import pyplot as plt
from calibrate import Calibrate

class Object_3d:

    CALIBRATION_IMAGE_PATH: str
    search_aruco_dict: None
    cal = None

    def __init__(self, path, search_aruco_dict):
        self.CALIBRATION_IMAGE_PATH = path
        self.search_aruco_dict = search_aruco_dict

        self.cal = Calibrate(self.CALIBRATION_IMAGE_PATH, self.search_aruco_dict)

if __name__ == "__main__":
    object_3d = Object_3d(path="calibration_images", search_aruco_dict=aruco.DICT_6X6_250)
    object_3d.cal.calibrate_camera()

    object_3d.cal.show_pose()