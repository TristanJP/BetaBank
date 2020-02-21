from detection import Detection
import cv2
from cv2 import aruco
from calibrate import Calibrate
from camera import Camera

class Frame_Analyser:

    CALIBRATION_IMAGE_PATH: str
    search_aruco_dict: None
    cal = None
    detection = None

    def __init__(self, path="calibration_images", search_aruco_dict=cv2.aruco.DICT_6X6_250):
        self.CALIBRATION_IMAGE_PATH = path
        self.search_aruco_dict = cv2.aruco.getPredefinedDictionary(search_aruco_dict)

        self.cal = Calibrate(path, search_aruco_dict)
        calibration_data = self.cal.calibrate_camera()
        self.detection = Detection(calibration_data)

    def analyse_video(self, video_path):
        print("test")

    def anaylse_frame(self, frame_path):
        frame = cv2.imread(frame_path)
        rvecs, tvecs, objPoints = self.detection.get_markers_in_frame(frame, self.search_aruco_dict)

        return rvecs, tvecs, objPoints

if __name__ == "__main__":

    frame_analyser = Frame_Analyser()
    frame_analyser.anaylse_frame("test_images/capture_10.png")

    #detection.get_markers_in_frame()
    #ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors = 
    #detection.cal.calibrate_camera()

    #object_3d.draw_object_3d()
    #detection.draw_cube_new()