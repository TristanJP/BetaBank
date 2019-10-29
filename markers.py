import cv2
from cv2 import aruco
import numpy as np
from matplotlib import pyplot as plt

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

fig = plt.figure()
nx = 4
ny = 3

