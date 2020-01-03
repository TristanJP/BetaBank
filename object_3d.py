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
        self.search_aruco_dict = cv2.aruco.getPredefinedDictionary(search_aruco_dict)

        self.cal = Calibrate(path, search_aruco_dict)

    def draw(self, frame, corners, ids):
        imgpts = np.int32(imgpts).reshape(-1,2)

        img = cv2.drawContours(img, [imgpts[:4]],-1, (0,255,0), -3)
        return img


    def draw_object_3d(self):
        cap = self.cal.capture_camera()

        while True:
            ret, frame = cap.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            parameters =  aruco.DetectorParameters_create()
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.search_aruco_dict,
                                                                parameters=parameters)
            # SUB PIXEL DETECTION
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
            for corner in corners:
                cv2.cornerSubPix(gray, corner, winSize = (3,3), zeroZone = (-1,-1), criteria = criteria)

            size_of_marker =  0.0125 # side lenght of the marker in meters
            rvecs, tvecs, objPoints = aruco.estimatePoseSingleMarkers(corners, size_of_marker , self.cal.camera_matrix, self.cal.distortion_coefficients0)

            length_of_axis = 0.025
            imaxis = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

            axis = np.float32([[3,0,0], [0,3,0], [0,0,3]]).reshape(-1,3)
            #imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, self.cal.camera_matrix, self.cal.distortion_coefficients0)
            #imaxis = self.draw(frame.copy(), corners, ids)

            if tvecs is not None:
                for i in range(len(tvecs)):
                    imaxis = aruco.drawAxis(imaxis, self.cal.camera_matrix, self.cal.distortion_coefficients0, rvecs[i], tvecs[i], length_of_axis)


            cv2.imshow("frame", imaxis)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cal.release_camera(cap)
                break

if __name__ == "__main__":
    object_3d = Object_3d(path="calibration_images", search_aruco_dict=cv2.aruco.DICT_6X6_250)
    #ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors = 
    object_3d.cal.calibrate_camera()

    object_3d.draw_object_3d()