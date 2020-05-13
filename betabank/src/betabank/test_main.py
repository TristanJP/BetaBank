import unittest
from .main import Main
import numpy as np
import cv2

class TestMain(unittest.TestCase):

    def test_rotation_matrix_to_euler_angles(self):
        main = Main()

        test_rvecs = np.array([20.0,10.0,0.0])
        R, _ = cv2.Rodrigues(test_rvecs)
        test_euler_rvecs = main.rotation_matrix_to_euler_angles(R)

        correct_euler_rvecs = np.array([-2.80809904, -0.16223459, 0.89993123])

        self.assertTrue(np.allclose(test_euler_rvecs, correct_euler_rvecs))