from detection import Detection
import cv2
from cv2 import aruco
from calibrate import Calibrate
from camera import Camera

class Frame_Analyser:

    CALIBRATION_IMAGE_PATH: str
    search_aruco_dict: None
    cal = None

    def __init__(self, frame, path="calibration_images", search_aruco_dict=cv2.aruco.DICT_6X6_250):
        self.CALIBRATION_IMAGE_PATH = path
        self.search_aruco_dict = cv2.aruco.getPredefinedDictionary(search_aruco_dict)

        self.cal = Calibrate(path, search_aruco_dict)


if __name__ == "__main__":

    cam = Camera()
    cam.start()

    while (True):
        frame = cam.current_frame
        cv2.imshow("frame", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            cam.release_camera()
            break
    
    #frame_analyser = Frame_Analyser()
    #detection = Detection()
    #detection.get_markers_in_frame()
    #ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors = 
    #detection.cal.calibrate_camera()

    #object_3d.draw_object_3d()
    #detection.draw_cube_new()