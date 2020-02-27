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
        frame_data = self.detection.get_markers_in_frame(frame, aruco_dict)
        
        return frame_data

    def center_of_mass(self, frame_data):
        total_x = 0
        total_y = 0

        frame_data_by_id = frame_data["ids"]

        for marker_id in frame_data_by_id:
            marker_corner = frame_data_by_id[marker_id]["corners"]
            total_x += (marker_corner[0] + marker_corner[2]) / 2
            total_y += (marker_corner[1] + marker_corner[5]) / 2

        size =  len(frame_data_by_id)
        average_x = total_x/size
        average_y = total_y/size

        return (average_x, average_y)

    def get_markers_position_relative_to_center(self, frame_data, center_of_mass):
        relative_vectors = {}
        frame_data_by_id = frame_data["ids"]

        for marker_id in frame_data_by_id:
            marker_corner = frame_data_by_id[marker_id]["corners"]
            relative_vector = ((marker_corner[0] - center_of_mass[0]), marker_corner[1] - center_of_mass[1])
            relative_vectors[marker_id] = relative_vector

        return relative_vectors

    def show_position(self, frame_path, position, corners, ids):
        frame = cv2.imread(frame_path)
        position = [int(pos) for pos in position]
        print(f"{position[0]}, {position[1]}")
        frame_new = cv2.line(frame, (position[0], position[1]), (position[0], position[1]), (65,65,255), 6)
        if corners is not None:
            frame_new = aruco.drawDetectedMarkers(frame_new, corners, ids)

        while True:
            cv2.imshow("frame", frame_new)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break  

if __name__ == "__main__":

    frame_analyser = Frame_Analyser()
    frame_path = "test_images/capture_10.png"

    frame_data = frame_analyser.anaylse_frame(frame_path, cv2.aruco.DICT_6X6_250)

    average_position = frame_analyser.center_of_mass(frame_data)

    frame_analyser.show_position(frame_path, average_position, None, None)

    relative_dict = frame_analyser.get_markers_position_relative_to_center(frame_data, average_position)

    exit()

    ids = frame_data["ids"]
    idc = tuple(ids.ravel())[0]
    corners = frame_data["corners"]
    corner = corners[0]
    corner = tuple(corner.ravel())

    relative_corner_x = relative_dict[idc][0]
    relative_corner_y = relative_dict[idc][1]

    #print(f"{relative_corner_x}, {relative_corner_y}")
    #print(f"{corner[0]}, {corner[1]}")

    relative_position = (relative_corner_x + corner[0], relative_corner_y + corner[1])
    test_pos = (corner[0], corner[1])

    frame_analyser.show_position(frame_path, test_pos, corners, ids)

    #detection.get_markers_in_frame()
    #ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors = 
    #detection.cal.calibrate_camera()

    #object_3d.draw_object_3d()
    #detection.draw_cube_new()