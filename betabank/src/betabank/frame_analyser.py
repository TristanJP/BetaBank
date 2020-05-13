from .detection import Detection
import cv2
from cv2 import aruco
from .calibrate import Calibrate
from .camera import Camera
import numpy as np
from .effects import Effects
import copy
import time

class Frame_Analyser:

    calibration_data: dict

    def __init__(self, calibration_data):
        self.calibration_data = calibration_data
        self.detection = Detection(self.calibration_data)

    # Returns dictionary of pose for all amrkers in a video.
    # Works per frame, finds which frame has the most markers detectable and returns
    # that frames pose data.
    def analyse_video(self, video_path, search_aruco_dict=cv2.aruco.DICT_6X6_250, show=False, total_markers=12):
        aruco_dict = cv2.aruco.getPredefinedDictionary(search_aruco_dict)

        cap = cv2.VideoCapture(video_path)
        delay = int((1/cap.get(5))*(1000/2))
        largest_frame_data = {"ids": {}}
        while(cap.isOpened()):
            ret, frame = cap.read()
            if not ret:
                break

            frame_data = self.anaylse_frame(frame)
            num_markers = len(frame_data["ids"])

            if num_markers > len(largest_frame_data["ids"]):
                largest_frame_data = frame_data.copy()

            if show:
                cv2.imshow('frame',frame)
                if (cv2.waitKey(delay) & 0xFF == ord('q')):
                    cap.release()
                    break

            if num_markers == total_markers:
                break
        return largest_frame_data

    # Returns dictionary of pose for all amrkers in view
    def anaylse_frame(self, frame, search_aruco_dict=cv2.aruco.DICT_6X6_250):
        aruco_dict = cv2.aruco.getPredefinedDictionary(search_aruco_dict)
        frame_data = self.detection.get_markers_in_frame(frame, aruco_dict)

        return frame_data

    # Returns the origin marker pose using board based detection
    def get_board_origin(self, frame, search_aruco_dict=cv2.aruco.DICT_6X6_250):
        aruco_dict = cv2.aruco.getPredefinedDictionary(search_aruco_dict)
        return self.detection.get_board_in_frame(frame, aruco_dict)

    # Used for testing
    # Gets a scale which can be used to convert between translation vectors and
    # real-world position relative to the camera (unit = centimetres)
    def get_scale(self, relative_frame_data, real_size=5.5):
        relative_tvecs = relative_frame_data[5]["relative_tvec"]
        dist = ((relative_tvecs[0])**2 + (relative_tvecs[1])**2 + (relative_tvecs[2])**2)**(0.5)
        return dist/real_size

    # Used for testing
    # Should return position for center of markers
    def center_of_mass_corners(self, frame_data):
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

    # Used for testing
    # Should get corners positoin relative to center
    def get_corners_relative_to_center(self, frame_data, center_of_mass):
        relative_vectors = {}
        frame_data_by_id = frame_data["ids"]

        for marker_id in frame_data_by_id:
            marker_corner = frame_data_by_id[marker_id]["corners"]
            relative_vector = ((marker_corner[0] - center_of_mass[0]), marker_corner[1] - center_of_mass[1])
            relative_vectors[marker_id] = relative_vector

        return relative_vectors

    # Inverts the perspective of the rotation and translation vectors which are
    # relative to the camera.
    def inverse_perspective(self, rvec, tvec):
        R, _ = cv2.Rodrigues(rvec)
        R = np.matrix(R).T
        inv_tvec = np.dot(-R, np.matrix(tvec))
        inv_rvec, _ = cv2.Rodrigues(R)
        return inv_rvec, inv_tvec

    def relative_position(self, rvec1, tvec1, rvec2, tvec2):
        # Gets (r/tvecs) relative to each other
        rvec1, tvec1 = rvec1.reshape((3, 1)), tvec1.reshape((3, 1))
        rvec2, tvec2 = rvec2.reshape((3, 1)), tvec2.reshape((3, 1))

        # Inverse the second marker, the right one in the image
        inv_rvec, inv_tvec = self.inverse_perspective(rvec2, tvec2)

        composed_data = cv2.composeRT(rvec1, tvec1, inv_rvec, inv_tvec)
        composed_rvec, composed_tvec = composed_data[0], composed_data[1]

        composed_rvec = composed_rvec.reshape((3, 1))
        composed_tvec = composed_tvec.reshape((3, 1))

        return composed_rvec, composed_tvec

    # Gets a dictionary of pose for all amrkers relative to the origin marker
    def get_relative_dict(self, frame_data, origin_marker_id):
        relative_frame_data = {}
        relative_frame_body = {}
        origin_rvec = frame_data["ids"][origin_marker_id]["marker_rvecs"]
        origin_tvec = frame_data["ids"][origin_marker_id]["marker_tvecs"]

        for marker_id in frame_data["ids"]:
            relative_rvec, relative_tvec = self.relative_position(origin_rvec, origin_tvec, frame_data["ids"][marker_id]["marker_rvecs"], frame_data["ids"][marker_id]["marker_tvecs"])

            relative_frame_body["relative_rvec"] = relative_rvec
            relative_frame_body["relative_tvec"] = relative_tvec
            relative_frame_data[marker_id] = relative_frame_body.copy()

        return relative_frame_data

    # Gets a dictionary of estiamted pose for the origin marker.
    # Effectively combines the relative pose dicitonary with the frame data from the
    # current frame to calculate where each marker believes the origin marker is
    # based on it's pose in the world-space.
    def get_combined_dict(self, frame_data, relative_data):
        combined_dict = {}
        combined_body = {}
        for marker_id in frame_data["ids"]:
            if marker_id not in relative_data:
                continue
            combined_data = cv2.composeRT(relative_data[marker_id]["relative_rvec"], relative_data[marker_id]["relative_tvec"], frame_data["ids"][marker_id]["marker_rvecs"].T, frame_data["ids"][marker_id]["marker_tvecs"].T)
            combined_body["combined_rvec"], combined_body["combined_tvec"] = combined_data[0], combined_data[1]
            combined_dict[marker_id] = combined_body.copy()

        return combined_dict

    # Averages the rotaiton and translation vectors to get a single value.
    # Used to get a pose for the origin marker from a dictionary of estimated pose/
    # Must also correct flipping of rotation vectors due to ambiguity in pose
    # estimation from solvePnP() method.
    def get_average_of_vectors(self, combined_frame_data):
        if len(combined_frame_data) >= 1:
            prev_vect = [0,0,0]
            sum_rvec = 0.0
            sum_tvec = 0.0
            for marker_id in combined_frame_data:
                rvec = combined_frame_data[marker_id]["combined_rvec"]
                tvec = combined_frame_data[marker_id]["combined_tvec"]

                # Correct flipping
                if (prev_vect[0] / rvec[0] < 0 and prev_vect[1] / rvec[1] < 0):
                    rvec[0] *= -1
                    rvec[1] *= -1
                    rvec[2] *= -1
                prev_vect = rvec
                sum_rvec += rvec
                sum_tvec += tvec

            size = len(combined_frame_data)
            average_rvec = sum_rvec/size
            average_tvec = sum_tvec/size

            return average_rvec, average_tvec

        return None, None

    # Returns the pose of the origin marker based on the current frame and
    # markers relative position.
    def find_origin_for_frame(self, frame, relative_frame_data):
        rt_frame_data = self.anaylse_frame(frame)
        combined_frame_data = self.get_combined_dict(rt_frame_data, relative_frame_data)
        average_rvec, average_tvec = self.get_average_of_vectors(combined_frame_data)
        return average_rvec, average_tvec

if __name__ == "__main__":

    camera = Camera()

    frame_analyser = Frame_Analyser(camera.get_calibration_data())

    frame = cv2.imread("test_images_1920x1080/capture_2.png")
    frame_data = frame_analyser.anaylse_frame(frame)
    relative_frame_data = frame_analyser.get_relative_dict(frame_data, 1)

    origin_rvec, origin_tvec =  frame_analyser.find_origin_for_frame(frame, relative_frame_data)
    print(origin_rvec)