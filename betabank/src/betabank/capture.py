import sys
import cv2
from cv2 import aruco
from .camera import Camera

class Capture:

    def __init__(self):
        self.cam = Camera()
        self.cam.start()

    def take_pictures(self, filename="capture_", image_folder="./src/betabank/ui/test_images_1920x1080", search_aruco_dict=cv2.aruco.DICT_6X6_250, grayscale=False, captureNum=0):
        search_aruco_dict = cv2.aruco.getPredefinedDictionary(search_aruco_dict)
        i = int(captureNum)

        while True:
            # Capture frame-by-frame
            frame = self.cam.current_frame
            ret = self.cam.successful_read
            if grayscale:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            else:
                gray = frame

            # get corners
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, search_aruco_dict)
            frame_markers = aruco.drawDetectedMarkers(gray.copy(), corners, ids)

            cv2.imshow('Capture Image', frame_markers)

            if cv2.waitKey(10) & 0xFF == ord(' '):
                print(f"{image_folder}/{filename[:-4]}_{i}{filename[-4:]}")
                cv2.imwrite(f"{image_folder}/{filename[:-4]}_{i}{filename[-4:]}", gray)
                i += 1

            if cv2.waitKey(10) & 0xFF == ord('q'):
                self.cam.release_camera()
                cv2.destroyAllWindows()
                break

    def take_video(self, filename, video_folder="./src/betabank/ui/test_videos_1920x1080", search_aruco_dict=cv2.aruco.DICT_6X6_250, grayscale=False):

        out = cv2.VideoWriter(f"{video_folder}/{filename}",cv2.VideoWriter_fourcc('m','p','4','v'), 20, (int(self.cam.video_capture.get(3)), int(self.cam.video_capture.get(4))))

        search_aruco_dict = cv2.aruco.getPredefinedDictionary(search_aruco_dict)

        while True:
            # Capture frame-by-frame
            frame = self.cam.current_frame
            ret = self.cam.successful_read
            if grayscale:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            else:
                gray = frame

            # get corners
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, search_aruco_dict)
            frame_markers = aruco.drawDetectedMarkers(gray.copy(), corners, ids)

            # Save frame
            out.write(gray)
            cv2.imshow('Capture Video', frame_markers)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.cam.release_camera()
                out.release()
                cv2.destroyAllWindows()
                break


if __name__ == "__main__":
    cap = Capture()
    if len(sys.argv) > 1:
        if sys.argv[1] == "picture":
            if len(sys.argv) > 2:
                if len(sys.argv) > 3:
                    if len(sys.argv) > 4:
                        cap.take_pictures(filename=sys.argv[2], image_folder=sys.argv[3], captureNum=sys.argv[4])
                    else:
                        cap.take_pictures(filename=sys.argv[2], image_folder=sys.argv[3])
                else:
                    cap.take_pictures(filename=sys.argv[2])
            else:
                cap.take_pictures()

        elif sys.argv[1] == "video":
            if len(sys.argv) > 2:
                if len(sys.argv) > 3:
                    cap.take_video(filename=sys.argv[2], video_folder=sys.argv[3])
                else:
                    cap.take_video(filename=sys.argv[2])
            else:
                cap.take_video("test1.mp4")
    else:
        # Defualt to taking pictures
        cap.take_pictures()

        # Calibration test
        #cap.take_pictures(False, "calibration_images_1920x1080", cv2.aruco.DICT_4X4_50)
