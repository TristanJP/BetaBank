import cv2
from cv2 import aruco
from detection import Detection
from camera import Camera
from frame_analyser import Frame_Analyser
from view import View

class Main():

    calibration_data: dict
    relative_frame_data: dict

    def __init__(self):
        print("running")
        self.cam = Camera()
        self.calibration_data = self.cam.get_calibration_data()
        self.cam.start()

        self.detection = Detection(self.calibration_data)
        self.frame_analyser = Frame_Analyser(self.calibration_data)
        self.view = View(self.calibration_data)

    def get_frame_data(self, input_data):
        if type(input_data) == str:
            file_type = input_data[-4:]
            if file_type in (".avi"):
                frame_data = self.frame_analyser.analyse_video(input_data)
            elif file_type in (".png", ".jpg"):
                image = cv2.imread(input_data)
                frame_data = self.frame_analyser.anaylse_frame(image)
            else:
                frame_data = None
        else:
            frame_data = self.frame_analyser.anaylse_frame(input_data)

        return frame_data

    def calculate_relative_dict(self, input_data, origin_marker_id):
        frame_data = self.get_frame_data(input_data)
        self.relative_frame_data = self.frame_analyser.get_relative_dict(frame_data, origin_marker_id)
        return self.relative_frame_data

    def run_realtime_relative(self, marker_id):
        input_data = "test_images/capture_0.png"  
        self.calculate_relative_dict(input_data, marker_id)

        while True:
            frame = self.cam.current_frame
            ret = self.cam.successful_read

            origin_rvec, origin_tvec = self.frame_analyser.find_origin_for_frame(frame, self.relative_frame_data)

            self.view.render_origin(frame, ret, origin_rvec, origin_tvec)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.cam.release_camera()
                break
    
    # JOIN THESE BASTARDS WITH CAM

    def run_video_relative(self, marker_id):
        video_path = "test_videos/test1.avi"  
        self.calculate_relative_dict(video_path, marker_id)

        cap = cv2.VideoCapture(video_path)
        delay = int((1/cap.get(5))*(1000/2))
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            origin_rvec, origin_tvec = self.frame_analyser.find_origin_for_frame(frame, self.relative_frame_data)

            self.view.render_origin(frame, ret, origin_rvec, origin_tvec)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                cap.release()
                break


if __name__ == "__main__":

    main = Main()

    #main.run_realtime_relative(1)
    main.run_video_relative(1)

    # main.camera.start()

    # while True:
    #     if cv2.waitKey(1) & 0xFF == ord('q'):
    #         break
        
    #     cv2.imshow("frame", main.camera.get_current_frame())
    
    # main.camera.release_camera()