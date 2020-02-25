from detection import Detection
import cv2
from cv2 import aruco
from calibrate import Calibrate
from camera import Camera

class Frame_Analyser:

    calibration_data: dict

    def __init__(self):
        self.calibrate()
        self.detection = Detection(self.calibration_data)

    def calibrate(self, path="calibration_images"):
        cal = Calibrate(path)
        self.calibration_data = cal.calibrate_camera()

    def analyse_video(self, video_path):
        print("test")

    def anaylse_frame(self, frame_path, search_aruco_dict=cv2.aruco.DICT_6X6_250):
        frame = cv2.imread(frame_path)
        rvecs, tvecs, objPoints = self.detection.get_markers_in_frame(frame, cv2.aruco.getPredefinedDictionary(search_aruco_dict))

        return rvecs, tvecs, objPoints

if __name__ == "__main__":

    frame_analyser = Frame_Analyser()
    frame_analyser.anaylse_frame("test_images/capture_10.png", cv2.aruco.DICT_6X6_250)

    #detection.get_markers_in_frame()
    #ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors = 
    #detection.cal.calibrate_camera()

    #object_3d.draw_object_3d()
    #detection.draw_cube_new()