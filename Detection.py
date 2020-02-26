import cv2
from cv2 import aruco
from camera import Camera
import copy

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
        while i < len(ids):
            marker_corner = tuple(corners[i].ravel())
            frame_data_body["corners"] = marker_corner
            frame_data_body["rvecs"] = rvecs[i]
            frame_data_body["tvecs"] = tvecs[i]

            frame_data[ids[i][0]] = frame_data_body.copy()

            i += 1

        frame_data["objPoints"] = objPoints
        return frame_data
