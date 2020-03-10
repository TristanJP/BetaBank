from detection import Detection
import cv2
from cv2 import aruco
from calibrate import Calibrate
from camera import Camera
import numpy as np
from camera import Camera
from effects import Effects
import copy

class Frame_Analyser:

    calibration_data: dict

    def __init__(self):
        self.calibrate()
        self.detection = Detection(self.calibration_data)

    def calibrate(self, path="calibration_images"):
        cal = Calibrate(path)
        self.calibration_data = cal.calibrate_camera()

    def analyse_video(self, video_path, search_aruco_dict=cv2.aruco.DICT_6X6_250, show=False):
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
            
            if (cv2.waitKey(delay) & 0xFF == ord('q')) or (num_markers == 12):
                cap.release()
                break
        
        return largest_frame_data

    def anaylse_frame(self, frame, search_aruco_dict=cv2.aruco.DICT_6X6_250):
        aruco_dict = cv2.aruco.getPredefinedDictionary(search_aruco_dict)
        frame_data = self.detection.get_markers_in_frame(frame, aruco_dict)
        
        return frame_data

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

    def get_corners_relative_to_center(self, frame_data, center_of_mass):
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
        effects = Effects()
        effects.render(image, mtx, dist, True, marker_rvecs, marker_tvecs, "axis")

        while True:
            cv2.imshow("frame", image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def render_video_relative(self, mtx, dist, relative_frame_data, video_path):
        effects = Effects()
        cap = cv2.VideoCapture(video_path)
        delay = int((1/cap.get(5))*(1000/2))
        while(cap.isOpened()):
            ret, frame = cap.read()
            if not ret:
                break

            rt_frame_data = self.anaylse_frame(frame)

            combined_frame_data = self.get_combined_dict(rt_frame_data, relative_frame_data)

            average_rvec, average_tvec = self.get_average_of_vectors(combined_frame_data)

            if average_tvec is not None:
                effects.render(frame, mtx, dist, ret, average_rvec, average_tvec, "axis")

            cv2.imshow("frame", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def render_realtime_by_marker_id(self, mtx, dist, marker_id):
        cam = Camera(self.calibration_data)
        cam.start()
        effects = Effects()

        while True:
            frame = cam.current_frame
            ret = cam.successful_read

            rt_frame_data = self.anaylse_frame(frame)

            if marker_id in rt_frame_data["ids"]:
                effects.render(frame, mtx, dist, ret, rt_frame_data["ids"][marker_id]["marker_rvecs"], rt_frame_data["ids"][marker_id]["marker_tvecs"], "axis")

            cv2.imshow("frame", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cam.release_camera()
                break

    def render_realtime_relative(self, mtx, dist, relative_frame_data):
        cam = Camera(self.calibration_data)
        cam.start()
        effects = Effects()

        while True:
            frame = cam.current_frame
            ret = cam.successful_read

            rt_frame_data = self.anaylse_frame(frame)

            combined_frame_data = self.get_combined_dict(rt_frame_data, relative_frame_data)

            average_rvec, average_tvec = self.get_average_of_vectors(combined_frame_data)

            if average_tvec is not None:
                effects.render(frame, mtx, dist, ret, average_rvec, average_tvec, "cube")

            cv2.imshow("frame", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cam.release_camera()
                break

    def inverse_perspective(self, rvec, tvec):
        R, _ = cv2.Rodrigues(rvec)
        R = np.matrix(R).T
        invTvec = np.dot(-R, np.matrix(tvec))
        invRvec, _ = cv2.Rodrigues(R)
        return invRvec, invTvec

    def relative_position(self, rvec1, tvec1, rvec2, tvec2):
        # Gets (r/tvecs) relative to each other

        rvec1, tvec1 = rvec1.reshape((3, 1)), tvec1.reshape((3, 1))
        rvec2, tvec2 = rvec2.reshape((3, 1)), tvec2.reshape((3, 1))

        # Inverse the second marker, the right one in the image
        invRvec, invTvec = self.inverse_perspective(rvec2, tvec2)

        #print(rvec2, tvec2, "\n and \n", self.inverse_perspective(invRvec, invTvec))

        data = cv2.composeRT(rvec1, tvec1, invRvec, invTvec)
        composedRvec, composedTvec = data[0], data[1]

        composedRvec = composedRvec.reshape((3, 1))
        composedTvec = composedTvec.reshape((3, 1))

        return composedRvec, composedTvec

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

    def get_combined_dict(self, frame_data, relative_data):
        combined_dict = {}
        combined_body = {}
        for marker_id in frame_data["ids"]:
            combined_data = cv2.composeRT(relative_data[marker_id]["relative_rvec"], relative_data[marker_id]["relative_tvec"], frame_data["ids"][marker_id]["marker_rvecs"].T, frame_data["ids"][marker_id]["marker_tvecs"].T)
            combined_body["combined_rvec"], combined_body["combined_tvec"] = combined_data[0], combined_data[1]
            combined_dict[marker_id] = combined_body.copy()

        return combined_dict

    def get_average_of_vectors(self, combined_frame_data):
        if len(combined_frame_data) >= 1:
            sum_rvec = 0.0
            sum_tvec = 0.0
            for marker_id in combined_frame_data:
                rvec = combined_frame_data[marker_id]["combined_rvec"]
                tvec = combined_frame_data[marker_id]["combined_tvec"]
                sum_rvec += rvec
                sum_tvec += tvec

            size = len(combined_frame_data)
            average_rvec = combined_frame_data[next(iter(combined_frame_data))]["combined_rvec"]
            average_tvec = sum_tvec/size

            return average_rvec, average_tvec

        return None, None

    def test_single_frame(self):
        # Get frame images etc.
        frame_path = "test_images/capture_10.png"
        alt_frame_path = "test_images/capture_14.png"
        image = cv2.imread(frame_path)
        alt_image = cv2.imread(alt_frame_path)

        # Get data about frame (tvecs, rvecs, corners)
        frame_data = self.anaylse_frame(image, cv2.aruco.DICT_6X6_250)
        alt_frame_data = self.anaylse_frame(alt_image, cv2.aruco.DICT_6X6_250)

        # Get relative t/rvecs for markers to 1st marker
        relative_frame_data = self.get_relative_dict(frame_data, 1)

        # Combine the new frame data with the relative data
        combined_frame_data = self.get_combined_dict(alt_frame_data, relative_frame_data)

        average_rvec, average_tvec = self.get_average_of_vectors(combined_frame_data)

        # Get new position based on realtive marker (r/tvecs) and new frame
        #relative_data = cv2.composeRT(relative_frame_data[2]["relative_rvec"], relative_frame_data[2]["relative_tvec"], alt_frame_data["ids"][2]["marker_rvecs"].T, alt_frame_data["ids"][2]["marker_tvecs"].T)

        # Draw the point
        self.render(alt_image, self.calibration_data["cam_mtx"], self.calibration_data["dist_coef"], average_rvec, average_tvec)

        #average_position = frame_analyser.center_of_mass(frame_data)
        #frame_analyser.show_position(frame_path, average_position, None, None)
        #relative_dict = frame_analyser.get_markers_position_relative_to_center(frame_data, average_position)

    def test_on_video(self):
        name = "test_videos/test60.avi"
        frame_data = self.analyse_video(name)

        chosen_marker = 1
        relative_frame_data = self.get_relative_dict(frame_data, chosen_marker)
        self.render_video_relative(self.calibration_data["cam_mtx"], self.calibration_data["dist_coef"], relative_frame_data, name)

    def test_realtime(self):
        if False:
            frame_path = "test_images/capture_10.png"
            image = cv2.imread(frame_path)
            frame_data = self.anaylse_frame(image, cv2.aruco.DICT_6X6_250)
        elif False:
            name = "test_videos/test40"
            frame_data = self.analyse_video(f"{name}.avi")
            np.save(f"{name}.npy", frame_data)
        else:
            name = "test_videos/test40"
            frame_data = np.load(f"{name}.npy",allow_pickle='TRUE').item()

        chosen_marker = 1
        relative_frame_data = self.get_relative_dict(frame_data, chosen_marker)

        #self.render_realtime_by_marker_id(self.calibration_data["cam_mtx"], self.calibration_data["dist_coef"], chosen_marker)
        self.render_realtime_relative(self.calibration_data["cam_mtx"], self.calibration_data["dist_coef"], relative_frame_data)


if __name__ == "__main__":
    
    frame_analyser = Frame_Analyser()

    #frame_analyser.test_single_frame()
    frame_analyser.test_realtime()
    #frame_analyser.test_on_video()

    #composedRvec, composedTvec = frame_analyser.relative_position(rt_frame_data["ids"][1]["marker_rvecs"], rt_frame_data["ids"][1]["marker_tvecs"], rt_frame_data["ids"][2]["marker_rvecs"], rt_frame_data["ids"][2]["marker_tvecs"])

    