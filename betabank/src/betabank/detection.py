import cv2
from cv2 import aruco
from .camera import Camera
import numpy as np
import copy
import math

class Detection():

    calibration_data: dict

    def __init__(self, calibration_data):
        self.calibration_data = calibration_data

    def get_markers_in_frame(self, frame, search_aruco_dict):
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        parameters =  aruco.DetectorParameters_create()

        # Find markers
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_frame, search_aruco_dict, parameters=parameters)

        # SUB PIXEL DETECTION
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
        for corner in corners:
            cv2.cornerSubPix(gray_frame, corner, winSize = (3,3), zeroZone = (-1,-1), criteria = criteria)

        size_of_marker =  0.0125 # side length of the marker in meters

        rvecs, tvecs, objPoints = aruco.estimatePoseSingleMarkers(corners,
                                                                  size_of_marker,
                                                                  self.calibration_data["cam_mtx"],
                                                                  self.calibration_data["dist_coef"])

        i = 0
        frame_data = {}
        frame_data_body = {}
        frame_data_ids = {}
        if ids is not None:
            while i < len(ids):
                #corrected_rvecs = self.correct_flipping(rvecs[i], tvecs[i])
                marker_corner = tuple(corners[i].ravel())
                frame_data_body["corners"] = marker_corner
                frame_data_body["marker_rvecs"] = rvecs[i]#corrected_rvecs[:]
                frame_data_body["marker_tvecs"] = tvecs[i]

                frame_data_ids[ids[i][0]] = frame_data_body.copy()

                i += 1

        frame_data["ids"] = frame_data_ids
        frame_data["objPoints"] = objPoints
        return frame_data

    def get_board_in_frame(self, frame, search_aruco_dict):
        markerLength = 0.04
        markerSeparation = 0.01
        markers_w = 7
        markers_h = 5

        grid_board = cv2.aruco.GridBoard_create(markers_w,
                                                markers_h,
                                                markerLength,
                                                markerSeparation,
                                                search_aruco_dict)

        corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, search_aruco_dict)

        if ids is not None and len(ids) > 0:
            retval, rvecs, tvecs = aruco.estimatePoseBoard(
                corners,
                ids,
                grid_board,
                self.calibration_data["cam_mtx"],
                self.calibration_data["dist_coef"], 0, 0)

            return tvecs, rvecs
        return np.array([0.0,0.0,0.0]), np.array([0.0,0.0,0.0])