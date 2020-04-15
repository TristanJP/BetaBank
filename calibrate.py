import cv2
import os
import numpy as np
from cv2 import aruco
from glob import glob

class Calibrate:
    calibrate_aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    search_aruco_dict: None
    board = cv2.aruco.CharucoBoard_create(7,9,0.025,0.0125, calibrate_aruco_dict)
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

    def get_calibration_images(self):
        fnames = glob(f"{self.CALIBRATION_IMAGE_PATH}/*.png")
        return fnames

    def read_chessboards(self):
        # Charuco based pose estimation.
        images = self.get_calibration_images()

        all_corners = []
        all_ids = []
        decimator = 0

        # SUB PIXEL CORNER DETECTION CRITERIA
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)

        for image in images:
            print(f"=> Processing image {image}")
            frame = cv2.imread(image)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, self.calibrate_aruco_dict)

            if len(corners) > 0:

                # SUB PIXEL DETECTION
                for corner in corners:
                    cv2.cornerSubPix(gray, corner,  winSize = (3,3),  zeroZone = (-1,-1), criteria = criteria)

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
            self.calibration_data = self.load_coefficients(self.CALIBRATION_STORED_DATA)
        else:
            print("Generating new calibration data...")
            # Calibrates the camera using the detected corners.
            all_corners, all_ids, imsize = self.read_chessboards()

            camera_matrix_init = np.array(
                [
                    [ 1000., 0., imsize[0]/2.],
                    [ 0., 1000., imsize[1]/2.],
                    [ 0., 0., 1.]
                ]
            )

            dist_coeffs_init = np.zeros((5,1))
            flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO)
            (ret, camera_matrix, distortion_coefficients,
            rotation_vectors, translation_vectors,
            standard_deviations_intrinsics, standard_deviations_extrinsics,
            per_view_errors) = cv2.aruco.calibrateCameraCharucoExtended(
                            charucoCorners=all_corners,
                            charucoIds=all_ids,
                            board=self.board,
                            imageSize=imsize,
                            cameraMatrix=camera_matrix_init,
                            distCoeffs=dist_coeffs_init,
                            flags=flags,
                            criteria=(cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-9))

            self.calibration_data = {"ret": ret, "cam_mtx": camera_matrix, "dist_coef": distortion_coefficients, "cam_rvecs": rotation_vectors, "cam_tvecs": translation_vectors}
            self.save_coefficients(f"{self.CALIBRATION_IMAGE_PATH}/{self.CALIBRATION_IMAGE_PATH}.yaml", camera_matrix, distortion_coefficients)

        print("DONE CALIBRATION")
        return self.calibration_data

    def save_coefficients(self, path, mtx, dist):
        mc_file = cv2.FileStorage(f"{path}", cv2.FILE_STORAGE_WRITE)
        mc_file.write("camera_matrix", mtx)
        mc_file.write("dist_coeff", dist)
        mc_file.release()

    def load_coefficients(self, path):
        mc_file = cv2.FileStorage(f"{path}", cv2.FILE_STORAGE_READ)
        camera_matrix = mc_file.getNode("camera_matrix").mat()
        dist_matrix = mc_file.getNode("dist_coeff").mat()
        mc_file.release()
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

# =================================
# Run Calibration

if __name__ == "__main__":

    # Calibrate based on images
    cal = Calibrate(path="calibration_images_1920x1080", search_aruco_dict=cv2.aruco.DICT_6X6_250)

    cal.calibrate_camera()

    undist = cal.get_undistorted_image(f"{cal.CALIBRATION_IMAGE_PATH}/capture_7.png")
    cal.show_images(undist)
