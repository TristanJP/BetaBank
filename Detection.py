import cv2
from cv2 import aruco
from Camera import Camera

class Detection():

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
        length_of_axis = 0.025

        rvecs, tvecs, objPoints = aruco.estimatePoseSingleMarkers(corners, size_of_marker , self.camera_matrix, self.distortion_coefficients0)

        print(rvecs)
        print(self.camera.calibration_data("rvecs"))

        
