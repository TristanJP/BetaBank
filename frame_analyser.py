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
        aruco_dict = cv2.aruco.getPredefinedDictionary(search_aruco_dict)
        corners, rvecs, tvecs, objPoints = self.detection.get_markers_in_frame(frame, aruco_dict)
        frame_data = {"corners": corners, "rvecs": rvecs, "tvecs": tvecs, "objPoints": objPoints}

        return frame_data

    def average_position(self, corners):
        total_x = 0
        total_y = 0

        for corner in corners:
            corner = tuple(corners[0].ravel())
            total_x += corner[0]
            total_y += corner[1]

        average_x = total_x/len(corners)
        average_y = total_y/len(corners)

        #print(f"{average_x}, {average_y}")
        return (int(average_x), int(average_y))

    def show_position(self, position):
        # cam = Camera()
        # cam.start()

        frame = cv2.imread("test_images/capture_10.png")
        print(f"{position[0]}, {position[1]}")
        frame_new = cv2.line(frame, (position[0], position[1]), (position[0], position[1]), (65,65,255), 10)

        while True:

            cv2.imshow("frame", frame_new)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break  

if __name__ == "__main__":

    frame_analyser = Frame_Analyser()
    frame_data = frame_analyser.anaylse_frame("test_images/capture_10.png", cv2.aruco.DICT_6X6_250)

    average_position = frame_analyser.average_position(frame_data["corners"])

    frame_analyser.show_position(average_position)

    #detection.get_markers_in_frame()
    #ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors = 
    #detection.cal.calibrate_camera()

    #object_3d.draw_object_3d()
    #detection.draw_cube_new()