import cv2
from cv2 import aruco
from camera import Camera

class Capture:

    def __init__(self):
        self.cam = Camera()
        self.cam.start()

    def take_pictures(self, grayscale, image_folder, search_aruco_dict=cv2.aruco.DICT_6X6_250):
        search_aruco_dict = cv2.aruco.getPredefinedDictionary(search_aruco_dict)
        i = 0

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

            cv2.imshow('frame', frame_markers)

            if cv2.waitKey(1) & 0xFF == ord(' '):
                
                cv2.imwrite(f"{image_folder}/capture_{i}.png", gray)
                i += 1

            if cv2.waitKey(1) & 0xFF == ord('q'):
                cam.release_camera()
                break
    
    def take_video(self, grayscale, video_folder, filename, search_aruco_dict=cv2.aruco.DICT_6X6_250):

        out = cv2.VideoWriter(f"{video_folder}/{filename}",cv2.VideoWriter_fourcc('M','J','P','G'), int(self.cam.video_capture.get(5)), (int(self.cam.video_capture.get(3)), int(self.cam.video_capture.get(4))))

        search_aruco_dict = cv2.aruco.getPredefinedDictionary(search_aruco_dict)
        i = 0

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

            cv2.imshow('frame', frame_markers)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.cam.release_camera()
                out.release()
                break


if __name__ == "__main__":
    
    cap = Capture()

    #cap.take_pictures(False, "test_images")

    cap.take_video(False, "test_videos", "test60.avi")