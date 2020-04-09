import time
import cv2
import os
from cv2 import aruco
import numpy as np
from glob import glob
from matplotlib import pyplot as plt

class Calibrate:
    calibrate_aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    search_aruco_dict: None
    board = cv2.aruco.CharucoBoard_create(7,9,.025,.0125, calibrate_aruco_dict)
    CALIBRATION_IMAGE_PATH: str
    CALIBRATION_STORED_DATA = None

    calibration_data: dict

    def __init__(self, path="calibration_images_1920x1080", search_aruco_dict=cv2.aruco.DICT_6X6_250):
        self.CALIBRATION_IMAGE_PATH = path
        self.search_aruco_dict = cv2.aruco.getPredefinedDictionary(search_aruco_dict)

        name = f"{path}.yaml"
        for root, dirs, files in os.walk(path):
            if name in files:
                self.CALIBRATION_STORED_DATA = os.path.join(root, name)      

    # OLD
    def capture_camera(self):
        cap = cv2.VideoCapture(0)
        cap.set(3, 1280)
        cap.set(4, 720)
        return cap

    # OLD
    def release_camera(self, cap):
        cap.release()
        cv2.destroyAllWindows()

    def get_calibration_images(self):
        fnames = glob(f"{self.CALIBRATION_IMAGE_PATH}/*.png")
        return fnames

    def read_chessboards(self):
        # Charuco base pose estimation.
        images = self.get_calibration_images()

        #print("POSE ESTIMATION STARTS:")
        all_corners = []
        all_ids = []
        decimator = 0
        # SUB PIXEL CORNER DETECTION CRITERION
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)

        for im in images:
            print(f"=> Processing image {im}")
            frame = cv2.imread(im)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, self.calibrate_aruco_dict)

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
        print("CAMERA CALIBRATION:")
        if self.CALIBRATION_STORED_DATA != None:
            print(f"  Found calibration data: {self.CALIBRATION_STORED_DATA}, loading...")
            self.calibration_data = self.loadCoefficients(self.CALIBRATION_STORED_DATA)
        else:
            print("Generating new calibration data...")
            # Calibrates the camera using the detected corners.
            all_corners, all_ids, imsize = self.read_chessboards()

            cameraMatrixInit = np.array([[ 1000.,    0., imsize[0]/2.],
                                        [    0., 1000., imsize[1]/2.],
                                        [    0.,    0.,           1.]])

            distCoeffsInit = np.zeros((5,1))
            flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO)
            #flags = (cv2.CALIB_RATIONAL_MODEL)
            (ret, camera_matrix, distortion_coefficients,
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

            self.calibration_data = {"ret": ret, "cam_mtx": camera_matrix, "dist_coef": distortion_coefficients, "cam_rvecs": rotation_vectors, "cam_tvecs": translation_vectors}
            self.saveCoefficients(f"{self.CALIBRATION_IMAGE_PATH}/{self.CALIBRATION_IMAGE_PATH}.yaml", camera_matrix, distortion_coefficients)

        print("DONE CALIBRATION")
        return self.calibration_data

    def saveCoefficients(self, path, mtx, dist):
        cv_file = cv2.FileStorage(f"{path}", cv2.FILE_STORAGE_WRITE)
        cv_file.write("camera_matrix", mtx)
        cv_file.write("dist_coeff", dist)
        # note you *release* you don't close() a FileStorage object
        cv_file.release()

    def loadCoefficients(self, path):
        # FILE_STORAGE_READ
        cv_file = cv2.FileStorage(f"{path}", cv2.FILE_STORAGE_READ)

        # note we also have to specify the type to retrieve other wise we only get a
        # FileNode object back instead of a matrix
        camera_matrix = cv_file.getNode("camera_matrix").mat()
        dist_matrix = cv_file.getNode("dist_coeff").mat()

        # Debug: print the values
        # print("camera_matrix : ", camera_matrix.tolist())
        # print("dist_matrix : ", dist_matrix.tolist())

        cv_file.release()
        return {"cam_mtx": camera_matrix, "dist_coef": dist_matrix}

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
        img_undist = cv2.undistort(frame, self.calibration_data["cam_mtx"], self.calibration_data["dist_coef"], None)
        return [frame, img_undist]

    def show_pose_for_image(self, image):
        frame = cv2.imread(f"{self.CALIBRATION_IMAGE_PATH}/{image}")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        parameters =  aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.search_aruco_dict,
                                                            parameters=parameters)
        # SUB PIXEL DETECTION
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
        for corner in corners:
            cv2.cornerSubPix(gray, corner, winSize = (3,3), zeroZone = (-1,-1), criteria = criteria)

        frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

        size_of_marker =  0.045 # side lenght of the marker in meters
        rvecs, tvecs, objPoints = aruco.estimatePoseSingleMarkers(corners, size_of_marker , self.camera_matrix, self.distortion_coefficients)

        length_of_axis = 0.1
        imaxis = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

        for i in range(len(tvecs)):
            imaxis = aruco.drawAxis(imaxis, self.camera_matrix, self.distortion_coefficients, rvecs[i], tvecs[i], length_of_axis)

        plt.figure()
        plt.imshow(imaxis)
        plt.grid()
        plt.show()

    # OLD
    def show_pose(self):
        cap = self.capture_camera()

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

            size_of_marker =  0.0125 # side length of the marker in meters
            rvecs, tvecs, objPoints = aruco.estimatePoseSingleMarkers(corners, size_of_marker , self.camera_matrix, self.distortion_coefficients)

            length_of_axis = 0.025
            imaxis = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

            if tvecs is not None:
                for i in range(len(tvecs)):
                    imaxis = aruco.drawAxis(frame, self.camera_matrix, self.distortion_coefficients, rvecs[i], tvecs[i], length_of_axis)

            cv2.imshow("frame", imaxis)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.release_camera(cap)
                break


# =================================
# Run Calibration

if __name__ == "__main__":

    # Calibrate based on images
    cal = Calibrate(path="calibration_images_1920x1080", search_aruco_dict=cv2.aruco.DICT_6X6_250)

    cal.calibrate_camera()

    undist = cal.get_undistorted_image(f"{cal.CALIBRATION_IMAGE_PATH}/capture_7.png")
    cal.show_images(undist)

    #cal.show_pose_for_image("capture_1.png")

    #cal.show_pose()


