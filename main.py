import cv2
import os
import asyncio
import threading
import websockets
from http.server import HTTPServer
from websocket import Websocket
from static_server import StaticServer
from cv2 import aruco
from detection import Detection
from camera import Camera
from frame_analyser import Frame_Analyser
import numpy as np
from view import View
import pygame
from PIL import Image
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
        for root, dirs, files in os.walk(path[0]):
            if new_name in files:
                print(f" Found previous data: {path[0]}/{new_name}, loading...")
                frame_data = np.load(f"{path[0]}/{new_name}", allow_pickle='TRUE').item()
                break

        if frame_data is None:
            if type(input_data) == str:
                print("  Generating Frame Data...")
                file_type = input_data[-4:]
                if file_type in (".avi"):
                    frame_data = self.frame_analyser.analyse_video(input_data)
                elif file_type in (".png", ".jpg"):
                    image = cv2.imread(input_data)
                    frame_data = self.frame_analyser.anaylse_frame(image)
                else:
                    frame_data = None
                np.save(f"{path[0]}/{new_name}", frame_data)
                print(f"  Saved data as: {path[0]}/{new_name}")
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
    def isRotationMatrix(self, R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype = R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    # Calculates rotation matrix to euler angles
    # The result is the same as MATLAB except the order
    # of the euler angles ( x and z are swapped ).
    def rotationMatrixToEulerAngles(self, R):

        assert(self.isRotationMatrix(R))
        
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
        self.calculate_relative_dict(self.v_path, 1)
        self.scale = main.frame_analyser.get_scale(self.relative_frame_data)

        while True:
            # GETTING FRAMES
            ret = self.cam.successful_read
            frame = self.cam.current_frame

            if ret:
                origin_tvecs, origin_rvecs = self.frame_analyser.get_board_origin(frame)

                R, _ = cv2.Rodrigues(origin_rvecs)
                origin_rvecs = self.rotationMatrixToEulerAngles(R)

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
    def run_magic(self, marker_id):
        print("\nRunning Realtime")

        v_frame_data = self.get_frame_data(self.v_path)

        self.calculate_relative_dict(self.v_path, 1)
        self.scale = main.frame_analyser.get_scale(self.relative_frame_data)

        #cap = cv2.VideoCapture("test_videos/test4.avi")
        #delay = int((1/cap.get(5))*(1000/2))
        delay = 1

        prev_rotation1 = None
        prev_rotation2 = None

        while True:
            self.current_state["use_board"] = False

            # GETTING FRAMES
            # frame = self.cam.current_frame
            ret = self.cam.successful_read
            frame = self.cam.current_frame
            #frame = cv2.imread("test_images/capture_0.png")
            
            # Getting video frames
            #v_ret, v_frame = cap.read()
            v_frame = cv2.imread(self.v_path)
            v_ret = True

            # GETTING DATA
            if ret and v_ret:
                #frame_undist = self.get_undistorted_image(frame)
                frame_undist = frame

                frame_data = self.frame_analyser.anaylse_frame(frame)
                v_frame_data = self.frame_analyser.anaylse_frame(v_frame)

                # Calc origin
                origin_rvec, origin_tvec = self.frame_analyser.find_origin_for_frame(frame, self.relative_frame_data)
                v_origin_rvec, v_origin_tvec = self.frame_analyser.find_origin_for_frame(v_frame, self.relative_frame_data)

                # GETTING UNDISTORTED IMAGE
                #frame_undist = self.get_undistorted_image(frame)

                # Encode frame to Base64
                bg = cv2.imencode(".jpg", frame_undist)
                bg = base64.b64encode(bg[1])
                bg = bg.decode("utf-8")
                self.current_state["frame"] = bg

                # Encode video frame to Base64
                if v_frame is not None:
                    vf = cv2.imencode(".jpg", v_frame)
                    vf = base64.b64encode(vf[1])
                    vf = vf.decode("utf-8")
                    self.current_state["v_frame"] = vf
            
                # OLD
                # RATIO STUFF
                # if False:
                #     if frame_data["ids"] is not None:
                #         if 5 in frame_data["ids"] and 1 in frame_data["ids"] and 5 in v_frame_data["ids"] and 1 in v_frame_data["ids"]:
                #             rel_pix_dataX = frame_data["ids"][5]["corners"][0] - frame_data["ids"][1]["corners"][0]
                #             rel_pix_dataY = frame_data["ids"][5]["corners"][1] - frame_data["ids"][1]["corners"][1]
                #             ratio1 = ((rel_pix_dataX)**2 + (rel_pix_dataY)**2)**(0.5)

                #             Trel_pix_dataX = v_frame_data["ids"][5]["corners"][0] - v_frame_data["ids"][1]["corners"][0]
                #             Trel_pix_dataY = v_frame_data["ids"][5]["corners"][1] - v_frame_data["ids"][1]["corners"][1]

                #             ratio2 = ((Trel_pix_dataX)**2 + (Trel_pix_dataY)**2)**(0.5)

                #             ratio = ratio1 / ratio2

                #             self.current_state["ratio"] = ratio

                    # rel_frame_data = self.frame_analyser.get_relative_dict(frame_data, 1)

                    # s1 = rel_frame_data[5]["relative_tvec"]
                    # s2 = self.relative_frame_data[5]["relative_tvec"]

                    # tratio1 = ((s1[0])**2 + (s1[1])**2 + (s1[2])**2)**(0.5)
                    # tratio2 = ((s2[0])**2 + (s2[1])**2 + (s2[2])**2)**(0.5)

                    # tratio = tratio1 / tratio2
            

                if False:
                    # Calcs distance of markers from camera
                    if origin_tvec is not None:
                        print(origin_tvec[2]/self.scale)


                # Send through websocket
                if origin_tvec is not None  and v_origin_tvec is not None:
                    R, _ = cv2.Rodrigues(origin_rvec)
                    origin_rvec = self.rotationMatrixToEulerAngles(R)

                    R2, _ = cv2.Rodrigues(v_origin_rvec)
                    v_origin_rvec = self.rotationMatrixToEulerAngles(R2)              
                    
                    origin_rvec = origin_rvec.flatten()
                    v_origin_rvec =  v_origin_rvec.flatten()

                    # OLD
                    # AVERAGE ROTATION ACROSS FRAMES (avoid flickering)
                    # if False:
                    #     # Ignore first frame as nothing to compare with
                    #     if prev_rotation1 is None:
                    #         prev_rotation1 = origin_rvec[:]
                    #     elif prev_rotation2 is None:
                    #         prev_rotation2 = prev_rotation1[:]
                    #         prev_rotation1 = origin_rvec[:]
                    #     else:
                    #         if (origin_rvec[0] / prev_rotation1[0] < 0  and origin_rvec[1] / prev_rotation1[1] < 0):
                    #             origin_rvec[0] *= -1
                    #             origin_rvec[1] *= -1

                    #         sum_rvecs = prev_rotation1 + prev_rotation2 + origin_rvec
                    #         avg_rvecs = sum_rvecs/3
                    #         prev_rotation2 = prev_rotation1[:]
                    #         prev_rotation1 = origin_rvec[:]
                    #         origin_rvec = avg_rvecs[:]


                    self.current_state["tvec"] = origin_tvec.flatten()
                    self.current_state["rvec"] = origin_rvec
                    self.current_state["scale"] = self.scale*100
                
                    self.current_state["v_origin_tvec"] = v_origin_tvec.flatten()
                    self.current_state["v_origin_rvec"] = v_origin_rvec               

            if cv2.waitKey(delay) & 0xFF == ord('q'):
                self.cam.release_camera()
                break

    def run_realtime_relative(self, marker_id):
        print("\nRunning Realtime")

        self.calculate_relative_dict(self.v_path, 1)

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

    def run_realtime_opengl_image(self, marker_id):
        print("\nRunning Realtime with OpenGL Image")
        while True:
            frame = self.cam.current_frame
            ret = self.cam.successful_read

            origin_rvec, origin_tvec = self.frame_analyser.find_origin_for_frame(frame, self.relative_frame_data)

            self.view.image_in_realtime(frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.cam.release_camera()
                break

    def run_image_opengl_image(self, image_path, marker_id):
        print("\nRunning Image with OpenGL Image")
        
        frame = cv2.imread(image_path)
        ret = True

        origin_rvec, origin_tvec = self.frame_analyser.find_origin_for_frame(frame, self.relative_frame_data)

        while True:
            self.view.image_in_image(frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.cam.release_camera()
                break

if __name__ == "__main__":

    main = Main()
    main.start()

    use_board = False

    main.v_path = "test_images_1920x1080/capture_1.png"

    if use_board:
        main.run_board()
    else:
        marker_id = 1

        # OLD
        ### Initial Frame Data
        #main.calculate_relative_dict("test_videos_1280x720/test4.avi", marker_id)

        ### Run
        main.run_realtime_relative(marker_id)
        #main.run_video_relative("test_videos/test2.avi", marker_id. False)
        #main.run_magic(1)