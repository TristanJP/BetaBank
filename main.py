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

    def run(self):
        input_data = "test_images/capture_0.png"  
        main.calculate_relative_dict(input_data, 1)

        while True:
            frame = self.cam.current_frame
            ret = self.cam.successful_read

            rt_frame_data = self.frame_analyser.anaylse_frame(frame)
            combined_frame_data = self.frame_analyser.get_combined_dict(rt_frame_data, self.relative_frame_data)
            average_rvec, average_tvec = self.frame_analyser.get_average_of_vectors(combined_frame_data)

            main.view.render_realtime_relative(frame, ret, main.relative_frame_data, average_rvec, average_tvec) # frame, ret, relative_frame_data, average_tvec, average_rvec

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.cam.release_camera()
                break


if __name__ == "__main__":

    main = Main()
    main.run()

    

    # main.camera.start()

    # while True:
    #     if cv2.waitKey(1) & 0xFF == ord('q'):
    #         break
        
    #     cv2.imshow("frame", main.camera.get_current_frame())
    
    # main.camera.release_camera()