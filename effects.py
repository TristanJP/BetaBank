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
            
            imgpts, _ = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
  
            # draw cube
            self._draw_axis(image, imgpts)

    def _draw_axis(self, img, imgpts):
        imgpts = np.int32(imgpts).reshape(-1,2)

        img = cv2.line(img, tuple(imgpts[0].ravel()), tuple(imgpts[3].ravel()), (255,0,0), 3)
        img = cv2.line(img, tuple(imgpts[0].ravel()), tuple(imgpts[1].ravel()), (0,255,0), 3)
        img = cv2.line(img, tuple(imgpts[0].ravel()), tuple(imgpts[4].ravel()), (0,0,255), 3)
  
    def _draw_cube(self, img, imgpts):
        imgpts = np.int32(imgpts).reshape(-1,2)
  
        # draw floor
        cv2.drawContours(img, [imgpts[:4]],-1,(0,0,200),-3)

        # draw pillars
        for i,j in zip(range(4),range(4,8)):
            cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]),(255),3)

        # draw roof
        cv2.drawContours(img, [imgpts[4:]],-1,(0,200,0),3)

        
  
        
  
        