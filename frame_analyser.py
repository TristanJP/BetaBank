from detection import Detection
import cv2
from cv2 import aruco
from calibrate import Calibrate
from camera import Camera
import numpy as np
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

    def render(self, image, mtx, dist, marker_rvecs, marker_tvecs):
        axis = np.float32([[0,0,0], [0.01,0,0], [0.01,0.01,0], [0,0.01,0],
                           [0,0,0.01],[0.01,0,0.01],[0.01,0.01,0.01],[0,0.01,0.01] ]).reshape(-1,3)
        imgpts, _ = cv2.projectPoints(axis, marker_rvecs, marker_tvecs, mtx, dist)

        imgpts = np.int32(imgpts).reshape(-1,2)

        image = cv2.line(image, (0,0), tuple(imgpts[0]), (125,255,65), 5)

        while True:
            cv2.imshow("frame", image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def inversePerspective(self, rvec, tvec):
        R, _ = cv2.Rodrigues(rvec)
        R = np.matrix(R).T
        invTvec = np.dot(-R, np.matrix(tvec))
        invRvec, _ = cv2.Rodrigues(R)
        return invRvec, invTvec

    def relativePosition(self, rvec1, tvec1, rvec2, tvec2):
        rvec1, tvec1 = rvec1.reshape((3, 1)), tvec1.reshape((3, 1))
        rvec2, tvec2 = rvec2.reshape((3, 1)), tvec2.reshape((3, 1))

        # Inverse the second marker, the right one in the image
        invRvec, invTvec = self.inversePerspective(rvec2, tvec2)

        print(rvec2, tvec2, "\n and \n", self.inversePerspective(invRvec, invTvec))

        info = cv2.composeRT(rvec1, tvec1, invRvec, invTvec)
        composedRvec, composedTvec = info[0], info[1]

        composedRvec = composedRvec.reshape((3, 1))
        composedTvec = composedTvec.reshape((3, 1))

        return composedRvec, composedTvec

if __name__ == "__main__":

    frame_analyser = Frame_Analyser()
    frame_path = "test_images/capture_10.png"

    frame_data = frame_analyser.anaylse_frame(frame_path, cv2.aruco.DICT_6X6_250)
    
    composedRvec, composedTvec = frame_analyser.relativePosition(frame_data["ids"][1]["marker_rvecs"], frame_data["ids"][1]["marker_tvecs"], frame_data["ids"][2]["marker_rvecs"][0], frame_data["ids"][2]["marker_tvecs"][0])

    image = cv2.imread(frame_path)

    stuff = cv2.composeRT(composedRvec, composedTvec, frame_data["ids"][2]["marker_rvecs"].T, frame_data["ids"][2]["marker_tvecs"].T)
    frame_analyser.render(image, frame_analyser.calibration_data["cam_mtx"], frame_analyser.calibration_data["dist_coef"], stuff[0], stuff[1])


    #average_position = frame_analyser.center_of_mass(frame_data)

    #frame_analyser.show_position(frame_path, average_position, None, None)

    #relative_dict = frame_analyser.get_markers_position_relative_to_center(frame_data, average_position)

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