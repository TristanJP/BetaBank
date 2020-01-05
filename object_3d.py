import time
import cv2
from cv2 import aruco
import numpy as np
from glob import glob
from matplotlib import pyplot as plt
from calibrate import Calibrate
import OpenGL
from OpenGL.GL import *
from OpenGL.GLU import *

class Object_3d:

    CALIBRATION_IMAGE_PATH: str
    search_aruco_dict: None
    cal = None

    def __init__(self, path, search_aruco_dict):
        self.CALIBRATION_IMAGE_PATH = path
        self.search_aruco_dict = cv2.aruco.getPredefinedDictionary(search_aruco_dict)

        self.cal = Calibrate(path, search_aruco_dict)

    def draw_plane(self, img, corners, imgpts):
        corner = tuple(corners[0].ravel())
        img = cv2.line(img, (corner[0], corner[1]), (corner[2], corner[3]), (255,125,65), 5)
        img = cv2.line(img, (corner[0], corner[1]), (corner[4], corner[5]), (255,125,65), 5)
        img = cv2.line(img, (corner[0], corner[1]), (corner[6], corner[7]), (255,125,65), 5)
        img = cv2.line(img, (corner[6], corner[7]), (corner[2], corner[3]), (255,125,65), 5)
        img = cv2.line(img, (corner[6], corner[7]), (corner[4], corner[5]), (255,125,65), 5)
        img = cv2.line(img, (corner[2], corner[3]), (corner[4], corner[5]), (255,125,65), 5)
        return img

    def draw_lines(self, img, corners, imgpts):
        corner = tuple(corners[0].ravel())
        img = cv2.line(img, (corner[0], corner[1]), (corner[2], corner[3]), (65,125,255), 5)
        img = cv2.line(img, (corner[0], corner[1]), (corner[4], corner[5]), (65,125,255), 5)
        img = cv2.line(img, (corner[0], corner[1]), (corner[6], corner[7]), (65,125,255), 5)
        return img

    def draw_point(self, img, corners, imgpnts):
        corner = tuple(corners[0].ravel())
        img = cv2.line(img, (0, 0), (0, 0), (125,255,65), 5)
        img = cv2.line(img, (corner[0], corner[1]), (corner[0], corner[1]), (125,255,65), 5)
        return img

    def Cube(self):
        vertices= (
            (1, -1, -1),
            (1, 1, -1),
            (-1, 1, -1),
            (-1, -1, -1),
            (1, -1, 1),
            (1, 1, 1),
            (-1, -1, 1),
            (-1, 1, 1)
            )
        edges = (
            (0,1),
            (0,3),
            (0,4),
            (2,1),
            (2,3),
            (2,7),
            (6,3),
            (6,4),
            (6,7),
            (5,1),
            (5,4),
            (5,7)
            )
        
        glBegin(GL_LINES)
        for edge in edges:
            for vertex in edge:
                glVertex3fv(vertices[vertex])
        #glEnd()
    
    def draw_cube(self, img, corners, imgptns):
        self.Cube()

    def _draw_cube(self, img, imgpts):
        imgpts = np.int32(imgpts).reshape(-1,2)
  
        # draw floor
        cv2.drawContours(img, [imgpts[:4]],-1,(200,150,10),-3)
  
        # draw pillars
        for i,j in zip(range(4),range(4,8)):
            cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]),(255),3)
  
        # draw roof
        cv2.drawContours(img, [imgpts[4:]],-1,(200,150,10),3)

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

            # set up criteria, object points and axis            
            #objp = np.zeros((6*7,3), np.float32)
            #objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
    
            #axis = np.float32([[0,0,0], [0,3,0], [3,3,0], [3,0,0],
                            #[0,0,-3],[0,3,-3],[3,3,-3],[3,0,-3] ])

            if tvecs is not None:
                for i in range(len(tvecs)):
                    # project 3D points to image plane
                    #cv2.cornerSubPix(gray,corners[i],(11,11),(-1,-1),criteria)
                    #rvecs, tvecs, _ = cv2.solvePnPRansac(objp, corners, self.cal.camera_matrix, self.cal.distortion_coefficients0)
                    #imgpts, _ = cv2.projectPoints(axis, rvecs, tvecs, self.cal.camera_matrix,  self.cal.distortion_coefficients0)
  
                    #imaxis = self.draw_point(imaxis, corners[i], ids)
                    #imaxis = self.draw_lines(imaxis, corners[i], ids)
                    imaxis = self.draw_plane(imaxis, corners[i], ids)
                    #imaxis = self._draw_cube(imaxis, corners[i])
                    imaxis = aruco.drawAxis(imaxis, self.cal.camera_matrix, self.cal.distortion_coefficients0, rvecs[i], tvecs[i], length_of_axis)

            cv2.imshow("frame", imaxis)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.cal.release_camera(cap)
                break

if __name__ == "__main__":
    object_3d = Object_3d(path="calibration_images", search_aruco_dict=cv2.aruco.DICT_6X6_250)
    #ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors = 
    object_3d.cal.calibrate_camera()

    object_3d.draw_object_3d()