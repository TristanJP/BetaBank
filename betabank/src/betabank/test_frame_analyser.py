import unittest
from frame_analyser import Frame_Analyser
from view import View
from camera import Camera
import numpy as np
import cv2

class TestFrameAnalyser(unittest.TestCase):

    def test_inverse_perspective(self):
        frame_analyser = Frame_Analyser(None)

        test_rvecs = np.array([20.0,10.0,0.0])
        test_tvecs = np.array([1.0,1.5,0.5])
        test_rvecs, test_tvecs = test_rvecs.reshape((3, 1)), test_tvecs.reshape((3, 1))

        inverse_rvecs, inverse_tvecs = frame_analyser.inverse_perspective(test_rvecs, test_tvecs)

        correct_inverse_rvecs = np.matrix([[2.47940714], [1.23970357], [8.52101665*(10**-16)]])
        correct_inverse_tvecs = np.matrix([[-1.85376063], [0.20752127], [0.14320068]])

        self.assertTrue(np.allclose(inverse_rvecs, correct_inverse_rvecs))
        self.assertTrue(np.allclose(inverse_tvecs, correct_inverse_tvecs))

    def test_on_frame(self):
        cam = Camera()
        view = View(cam.get_calibration_data())
        frame_analyser = Frame_Analyser(cam.get_calibration_data())

        # Get frame images etc.
        frame_path = "test_images_1920x1080/capture_0.png"
        alt_frame_path = "test_images_1920x1080/capture_1.png"
        frame = cv2.imread(frame_path)
        alt_frame = cv2.imread(alt_frame_path)

        # Get data about frame (tvecs, rvecs, corners)
        frame_data = frame_analyser.anaylse_frame(frame, cv2.aruco.DICT_6X6_250)

        # Get relative t/rvecs for markers to 1st marker
        relative_frame_data = frame_analyser.get_relative_dict(frame_data, 1)

        # Get origin marker pose
        average_rvec, average_tvec = frame_analyser.find_origin_for_frame(alt_frame, relative_frame_data)

        correct_average_rvec = np.matrix([[1.92375314], [-2.15433403], [-0.50521701]])
        correct_average_tvec = np.matrix([[0.00292129], [0.0002342], [0.18293143]])

        self.assertTrue(np.allclose(average_rvec, correct_average_rvec))
        self.assertTrue(np.allclose(average_tvec, correct_average_tvec))

        # Draw the point
        while True:
            view.render_origin(alt_frame, True, average_rvec, average_tvec)

            # Set hide to False to manually view the estimated pose
            hide = True
            if (cv2.waitKey(1) & 0xFF == ord('q') or hide):
                break