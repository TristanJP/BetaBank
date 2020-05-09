from http.server import HTTPServer
from websocket import Websocket
from static_server import StaticServer
from cv2 import aruco
from detection import Detection
from camera import Camera
from frame_analyser import Frame_Analyser
from view import View
from PIL import Image

import cv2
import os
import sys
import asyncio
import threading
import websockets
import numpy as np
import base64
import math

class Main():

    calibration_data: dict
    relative_frame_data: dict

    current_state = dict()

    def __init__(self):
        print("Starting...\n")
        self.cam = Camera()
        self.calibration_data = self.cam.get_calibration_data()
        self.cam.start()

        self.detection = Detection(self.calibration_data)
        self.frame_analyser = Frame_Analyser(self.calibration_data)
        self.view = View(self.calibration_data)

    def start(self):
        ui = threading.Thread(target=self.start_ui, daemon=True)
        ui.start()

        websocket = threading.Thread(target=self.start_websocket, daemon=True)
        websocket.start()

    def start_websocket(self, port=8081):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        ws = Websocket(self.current_state)
        start_server = websockets.serve(ws.push_state, '0.0.0.0', port)
        print('Created websocket server')
        loop.create_task(ws.broadcast_state(1/30))
        loop.run_until_complete(start_server)
        loop.run_forever()

    def start_ui(self, server_class=HTTPServer, handler_class=StaticServer, port=8080):
        server_address = ('', port)
        httpd = server_class(server_address, handler_class)
        print('Starting httpd on port', port)
        httpd.serve_forever()

    def get_frame_data(self, input_data):
        frame_data = None
        print("\nGetting Init Frame Data:")
        path = input_data.split("/")
        new_name = f"{path[1][:-4]}_FD.npy"
        for root, dirs, files in os.walk("ui/"+path[0]):
            if new_name in files:
                print(f" Found previous data: ui/{path[0]}/{new_name}, loading...")
                frame_data = np.load(f"ui/{path[0]}/{new_name}", allow_pickle='TRUE').item()
                break

        if frame_data is None:
            if type(input_data) == str:
                print("  Generating Frame Data...")
                file_type = input_data[-4:]
                if file_type in (".avi"):
                    frame_data = self.frame_analyser.analyse_video("ui/"+input_data)
                elif file_type in (".png", ".jpg"):
                    image = cv2.imread(input_data)
                    frame_data = self.frame_analyser.anaylse_frame("ui/"+image)
                else:
                    frame_data = None
                np.save(f"ui/{path[0]}/{new_name}", frame_data)
                print(f"  Saved data as: ui/{path[0]}/{new_name}")
            elif False:
                # Do not support frames as input
                frame_data = self.frame_analyser.anaylse_frame(input_data)
            else:
                print("  Frame Data failed!")
        print("DONE FRAME DATA")
        return frame_data

    def calculate_relative_dict(self, input_data, origin_marker_id):
        frame_data = self.get_frame_data(input_data)
        self.relative_frame_data = self.frame_analyser.get_relative_dict(frame_data, origin_marker_id)
        return self.relative_frame_data

    def get_undistorted_image(self, frame) -> list:
        img_undist = cv2.undistort(frame, self.calibration_data["cam_mtx"], self.calibration_data["dist_coef"], None)
        return img_undist

    # Checks if a matrix is a valid rotation matrix.
    def is_rotation_matrix(self, rotation_matrix):
        rotation_matrix_transposed = np.transpose(rotation_matrix)
        potential_identity = np.dot(rotation_matrix_transposed, rotation_matrix)
        identity = np.identity(3, dtype = rotation_matrix.dtype)
        n = np.linalg.norm(identity - potential_identity)
        return n < 1e-6

    # Converts rotation matrix to euler angles
    def rotation_matrix_to_euler_angles(self, R):

        assert(self.is_rotation_matrix(R))

        sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

        singular = sy < 1e-6

        if  not singular :
            x = math.atan2(R[2,1] , R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else :
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0

        return np.array([x, y, z])

    # RUN ZONE

    # BOARD
    def run_board(self):
        self.calculate_relative_dict(self.relative_capture_path, 1)
        self.scale = main.frame_analyser.get_scale(self.relative_frame_data)

        while True:
            # GETTING FRAMES
            ret = self.cam.successful_read
            frame = self.cam.current_frame

            if ret:
                origin_tvecs, origin_rvecs = self.frame_analyser.get_board_origin(frame)

                R, _ = cv2.Rodrigues(origin_rvecs)
                origin_rvecs = self.rotation_matrix_to_euler_angles(R)

                origin_rvecs = origin_rvecs.flatten()
                origin_tvecs = origin_tvecs.flatten()

                self.current_state["tvec"] = origin_tvecs
                self.current_state["rvec"] = origin_rvecs

                self.current_state["use_board"] = True
                self.current_state["scale"] = self.scale*100

                bg = cv2.imencode(".jpg", frame)
                bg = base64.b64encode(bg[1])
                bg = bg.decode("utf-8")
                self.current_state["frame"] = bg

    # SINGLE MARKERS
    def run_webgl(self, marker_id=1, relative_source_path="test_images_1920x1080/capture_2.png", display_path="test_videos_1920x1080/demo2.mp4", use_picture=False):
        print("\nRunning Realtime")

        self.current_state["path"] = display_path

        # GET RELATIVE INFO AND SCALE
        self.calculate_relative_dict(relative_source_path, marker_id)
        self.scale = main.frame_analyser.get_scale(self.relative_frame_data)
        self.current_state["scale"] = self.scale*100

        delay = 1

        while True:
            self.current_state["use_picture"] = (use_picture in ["True", "true", "1"])

            # GETTING FRAMES
            frame = self.cam.current_frame
            ret = self.cam.successful_read

            # GETTING DATA
            if ret:
                frame_undist = frame

                frame_data = self.frame_analyser.anaylse_frame(frame)

                # Calc origin
                origin_rvec, origin_tvec = self.frame_analyser.find_origin_for_frame(frame, self.relative_frame_data)

                # Encode frame to Base64
                bg = cv2.imencode(".jpg", frame_undist)
                bg = base64.b64encode(bg[1])
                bg = bg.decode("utf-8")
                self.current_state["frame"] = bg


                if False:
                    # Calcs distance of markers from camera
                    if origin_tvec is not None:
                        print(f"{origin_tvec[0]/self.scale} | {origin_tvec[1]/self.scale} | {origin_tvec[2]/self.scale}")


                # Send through websocket
                if origin_tvec is not None:
                    R, _ = cv2.Rodrigues(origin_rvec)
                    origin_rvec = self.rotation_matrix_to_euler_angles(R)

                    origin_rvec = origin_rvec.flatten()

                    self.current_state["tvec"] = origin_tvec.flatten()
                    self.current_state["rvec"] = origin_rvec

            if cv2.waitKey(delay) & 0xFF == ord('q'):
                self.cam.release_camera()
                break

    def run_realtime_relative(self, marker_id):
        print("\nRunning Realtime")

        self.calculate_relative_dict(self.relative_capture_path, marker_id)

        while True:
            frame = self.cam.current_frame
            ret = self.cam.successful_read

            origin_rvec, origin_tvec = self.frame_analyser.find_origin_for_frame(frame, self.relative_frame_data)

            self.view.render_origin(frame, ret, origin_rvec, origin_tvec)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.cam.release_camera()
                break

    def run_video_relative(self, video_path, marker_id, loop=False):
        print("\nRunning Video")
        cap = cv2.VideoCapture(video_path)
        delay = int((1/cap.get(5))*(1000/3))
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                if loop:
                    cap.set(2, 0)
                    ret, frame = cap.read()
                else:
                    cap.release()
                    break

            origin_rvec, origin_tvec = self.frame_analyser.find_origin_for_frame(frame, self.relative_frame_data)

            self.view.render_origin(frame, ret, origin_rvec, origin_tvec)

            if cv2.waitKey(delay) & 0xFF == ord('q'):
                cap.release()
                break

if __name__ == "__main__":

    main = Main()
    main.start()

    if len(sys.argv) > 1:
        if sys.argv[1] == "webgl":
            if len(sys.argv) > 2:
                if len(sys.argv) > 3:
                        if len(sys.argv) > 4:
                            main.run_webgl(relative_source_path=sys.argv[2], display_path=sys.argv[3], use_picture=sys.argv[4])
                        else:
                            main.run_webgl(relative_source_path=sys.argv[2], display_path=sys.argv[3])
                else:
                    main.run_webgl(relative_source_path=sys.argv[2])
            else:
                main.run_webgl()
        # elif sys.argv[1] == "realtime":
        #     if len(sys.argv) > 2:
        #         if len(sys.argv) > 3:
        #             cap.take_video(filename=sys.argv[2], video_folder=sys.argv[3])
        #         else:
        #             cap.take_video(filename=sys.argv[2])
        #     else:
        #         cap.take_video("test3.avi")
        # elif sys.argv[1] == "video":
        #     if len(sys.argv) > 2:
        #         if len(sys.argv) > 3:
        #             cap.take_video(filename=sys.argv[2], video_folder=sys.argv[3])
        #         else:
        #             cap.take_video(filename=sys.argv[2])
        #     else:
        #         cap.take_video("test3.avi")
    else:
        # Default to ThreeJS

        marker_id = 1

        # OLD
        ### Initial Frame Data
        #main.calculate_relative_dict("test_videos_1280x720/test4.avi", marker_id)

        ### Run
        #main.run_realtime_relative(marker_id)
        #main.run_video_relative("test_videos_1280x720/test2.avi", marker_id. False)
        main.run_webgl(marker_id)