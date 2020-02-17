import cv2
import numpy as np

class Effects(object):
    
    def render(self, image, mtx, dist, ret, corners, rvecs, tvecs, objPoints):
  
  
        # set up criteria, object points and axis
          
        objp = np.zeros((6*7,3), np.float32)
        objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
  
        axis = np.float32([[0,0,0], [0,3,0], [3,3,0], [3,0,0],
                           [0,0,-3],[0,3,-3],[3,3,-3],[3,0,-3] ])
  
        # find grid corners in image
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

  
        if ret == True:
              
            # project 3D points to image plane
            #rvecs, tvecs, _ = cv2.solvePnPRansac(objp, corners, mtx, dist)
            imgpts, _ = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
  
            # draw cube
            self._draw_cube(image, imgpts)
  
    def _draw_cube(self, img, imgpts):
        imgpts = np.int32(imgpts).reshape(-1,2)
  
        # draw floor
        cv2.drawContours(img, [imgpts[:4]],-1,(0.02,0.015,0.001),-3)
  
        # draw pillars
        for i,j in zip(range(4),range(4,8)):
            cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]),(255),3)
  
        # draw roof
        cv2.drawContours(img, [imgpts[4:]],-1,(0.02,0.015,0.001),3)