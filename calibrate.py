import time
import cv2
from cv2 import aruco
import numpy as np
from glob import glob

class Calibrate:
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board = cv2.aruco.CharucoBoard_create(7,9,.025,.0125, aruco_dict)
    CALIBRATION_IMAGE_PATH: str

    def __init__(self, path):
        self.CALIBRATION_IMAGE_PATH = path

    # CREATE CHARUCO BOARD
    def generate_charuco_board(self):
        img = self.board.draw((200*3,200*3))

        #Dump the calibration board to a file
        cv2.imwrite('images/charuco.png',img)

    def capture_camera(self):
        cap = cv2.VideoCapture(0)
        cap.set(3, 1280)
        cap.set(4, 720)
        return cap

    def release_camera(self, cap):
        cap.release()
        cv2.destroyAllWindows()

    def take_pictures(self):
        cap = self.capture_camera()
        i = 0

        while True:
            # Capture frame-by-frame
            ret, frame = cap.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # get corners
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict)
            frame_markers = aruco.drawDetectedMarkers(gray.copy(), corners, ids)

            cv2.imshow('frame',frame_markers)


            if cv2.waitKey(1) & 0xFF == ord(' '):
                
                cv2.imwrite(f"calibration_images/capture_{i}.png", gray)
                i += 1

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.release_camera(cap)

    def get_calibration_images(self):

        fnames = glob(f"{self.CALIBRATION_IMAGE_PATH}/*.png")
        #fnames = [f for f in glob("calibration_images/*.png")]
        return fnames
        print(fnames)


    def read_chessboards(self):
        # Charuco base pose estimation.
        images = self.get_calibration_images()

        print("POSE ESTIMATION STARTS:")
        all_corners = []
        all_ids = []
        decimator = 0
        # SUB PIXEL CORNER DETECTION CRITERION
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)

        for im in images:
            print(f"=> Processing image {im}")
            frame = cv2.imread(im)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, self.aruco_dict)

            if len(corners)>0:
                # SUB PIXEL DETECTION
                for corner in corners:
                    cv2.cornerSubPix(gray, corner,
                                    winSize = (3,3),
                                    zeroZone = (-1,-1),
                                    criteria = criteria)
                res2 = cv2.aruco.interpolateCornersCharuco(corners,ids,gray,self.board)
                if res2[1] is not None and res2[2] is not None and len(res2[1])>3 and decimator%1==0:
                    all_corners.append(res2[1])
                    all_ids.append(res2[2])

            decimator+=1

        imsize = gray.shape
        return all_corners, all_ids, imsize

    def calibrate_camera(self):
        # Calibrates the camera using the dected corners.

        all_corners, all_ids, imsize = self.read_chessboards()
        
        print("CAMERA CALIBRATION")

        cameraMatrixInit = np.array([[ 1000.,    0., imsize[0]/2.],
                                    [    0., 1000., imsize[1]/2.],
                                    [    0.,    0.,           1.]])

        distCoeffsInit = np.zeros((5,1))
        flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO)
        #flags = (cv2.CALIB_RATIONAL_MODEL)
        (ret, camera_matrix, distortion_coefficients0,
        rotation_vectors, translation_vectors,
        stdDeviationsIntrinsics, stdDeviationsExtrinsics,
        perViewErrors) = cv2.aruco.calibrateCameraCharucoExtended(
                        charucoCorners=all_corners,
                        charucoIds=all_ids,
                        board=self.board,
                        imageSize=imsize,
                        cameraMatrix=cameraMatrixInit,
                        distCoeffs=distCoeffsInit,
                        flags=flags,
                        criteria=(cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-9))

        return ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors

    # show images
    def show_images(self, images: list):

        while True:
            i = 0
            for image in images:
                cv2.imshow(f"frame{i}", image)
                i += 1

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def get_undistorted_image(self, fname: str) -> list:
        frame = cv2.imread(fname)
        img_undist = cv2.undistort(frame,camera_matrix,distortion_coefficients0,None)
        return [frame, img_undist]


# =================================
# Run Calibration

cal = Calibrate("calibration_images")

#cal.take_pictures()

ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors = cal.calibrate_camera()

undist = cal.get_undistorted_image(f"{cal.CALIBRATION_IMAGE_PATH}/capture_3.png")

cal.show_images(undist)

