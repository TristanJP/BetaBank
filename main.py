import cv2
from cv2 import aruco
from Detection import Detection
from Camera import Camera

class Main():

    def __init__(self):
        print("running")
        self.camera = Camera()
        self.detection = Detection()


if __name__ == "__main__":
    main = Main()

    main.camera.start()

    while True:
        cv2.imshow(main.camera.get_current_frame())
    
    #main.detection.get_markers_in_frame(main.camera.get_current_frame(), cv2.aruco.DICT_6X6_250)

    #main.camera.release_camera()