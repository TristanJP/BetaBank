import cv2
import os
from cv2 import aruco
from detection import Detection
from camera import Camera
from frame_analyser import Frame_Analyser
import numpy as np
from view import View

class Main():

    calibration_data: dict
    relative_frame_data: dict

    def __init__(self):
        print("running\n")
        self.cam = Camera()
        self.calibration_data = self.cam.get_calibration_data()
        self.cam.start()

        self.detection = Detection(self.calibration_data)
        self.frame_analyser = Frame_Analyser(self.calibration_data)
        self.view = View(self.calibration_data)

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

    def run_realtime_relative(self, marker_id):
        print("\nRunning Realtime")
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

if __name__ == "__main__":

    main = Main()
    marker_id = 1

    ### Initial Frame Data
    main.calculate_relative_dict("test_images/capture_0.png" , marker_id)
    #main.calculate_relative_dict("test_videos/test2.avi", marker_id)

    ### Run
    #main.run_realtime_relative(marker_id)
    #main.run_video_relative("test_videos/test2.avi", marker_id. False)
    main.run_realtime_opengl_image(marker_id)




    # main.camera.start()

    # while True:
    #     if cv2.waitKey(1) & 0xFF == ord('q'):
    #         break
        
    #     cv2.imshow("frame", main.camera.get_current_frame())
    
    # main.camera.release_camera()