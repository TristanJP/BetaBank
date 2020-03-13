from detection import Detection
import cv2
from cv2 import aruco
from calibrate import Calibrate
from camera import Camera
import numpy as np
from camera import Camera
from effects import Effects
from frame_analyser import Frame_Analyser
import copy

import pygame
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *

class View:

    calibration_data: dict
    frame_analyser: Frame_Analyser

    def __init__(self, calibration_data, frame_analyser = None, camera = None):
        self.calibration_data = calibration_data
        self.frame_analyser = frame_analyser
        self.cam = camera

    def image_in_image(self, master_image, child_image, master_frame_data):
        print("image_in_image")

    def image_in_video(self, ):
        print("image_in_video")

    def image_in_realtime(self):
        print("image_in_realtime")

    def video_in_video(self):
        print("video_in_video")

    def video_in_realtime(self):
        print("video_in_realtime")

    def render_origin(self, frame, ret, origin_rvec, origin_tvec):
        effects = Effects()

        if origin_tvec is not None:
            effects.render(frame, self.calibration_data["cam_mtx"], self.calibration_data["dist_coef"], ret, origin_rvec, origin_tvec, "cube")
        cv2.imshow("frame", frame)
