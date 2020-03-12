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

    def render(self, image, marker_rvecs, marker_tvecs, shape):
        effects = Effects()
        effects.render(image, self.calibration_data["cam_mtx"], self.calibration_data["dist_coef"], True, marker_rvecs, marker_tvecs, shape)

        while True:
            cv2.imshow("frame", image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def render_video_relative(self, relative_frame_data, video_path):
        effects = Effects()
        cap = cv2.VideoCapture(video_path)
        delay = int((1/cap.get(5))*(1000/2))
        while(cap.isOpened()):
            ret, frame = cap.read()
            if not ret:
                break

            rt_frame_data = self.anaylse_frame(frame)

            combined_frame_data = self.get_combined_dict(rt_frame_data, relative_frame_data)

            average_rvec, average_tvec = self.get_average_of_vectors(combined_frame_data)

            if average_tvec is not None:
                effects.render(frame, self.calibration_data["cam_mtx"], self.calibration_data["dist_coef"], ret, average_rvec, average_tvec, "axis")

            cv2.imshow("frame", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def render_realtime_by_marker_id(self, marker_id):
        cam = Camera(self.calibration_data)
        cam.start()
        effects = Effects()

        while True:
            frame = cam.current_frame
            ret = cam.successful_read

            rt_frame_data = self.anaylse_frame(frame)

            if marker_id in rt_frame_data["ids"]:
                effects.render(frame, self.calibration_data["cam_mtx"], self.calibration_data["dist_coef"], ret, rt_frame_data["ids"][marker_id]["marker_rvecs"], rt_frame_data["ids"][marker_id]["marker_tvecs"], "axis")

            cv2.imshow("frame", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cam.release_camera()
                break

    def render_realtime_relative(self, frame, ret, relative_frame_data, average_rvec, average_tvec):
        effects = Effects()

        # while True:
        #     frame = self.cam.current_frame
        #     ret = self.cam.successful_read

            # rt_frame_data = self.frame_analyser.anaylse_frame(frame)
            # combined_frame_data = self.frame_analyser.get_combined_dict(rt_frame_data, relative_frame_data)
            # average_rvec, average_tvec = self.frame_analyser.get_average_of_vectors(combined_frame_data)

        if average_tvec is not None:
            effects.render(frame, self.calibration_data["cam_mtx"], self.calibration_data["dist_coef"], ret, average_rvec, average_tvec, "cube")
        cv2.imshow("frame", frame)

            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     cam.release_camera()
            #     break


if __name__ == "__main__":
    view = View()

    view.render()

    #view.image_in_image(master_image_path, child_image_path)