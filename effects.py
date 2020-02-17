import cv2
import numpy as np

class Effects(object):
    
    def render(self, image, mtx, dist, ret, corners, rvecs, tvecs, objPoints):
  
  
        # set up criteria, object points and axis
  
        axis = np.float32([[0,0,0], [0.01,0,0], [0.01,0.01,0], [0,0.01,0],
                           [0,0,0.01],[0.01,0,0.01],[0.01,0.01,0.01],[0,0.01,0.01] ]).reshape(-1,3)
  
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
        cv2.drawContours(img, [imgpts[:4]],-1,(0,0,200),-3)

        # draw pillars
        for i,j in zip(range(4),range(4,8)):
            cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]),(255),3)

        # draw roof
        cv2.drawContours(img, [imgpts[4:]],-1,(0,200,0),3)

        
  
        
  
        