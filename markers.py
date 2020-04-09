import sys
import cv2
from cv2 import aruco
import matplotlib as mpl
from matplotlib import pyplot as plt

def generate_aruco_board():
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

    figure = plt.figure()
    x = 4
    y = 3

    for i in range(1, x*y+1):
        grid = figure.add_subplot(y,x, i)
        image = aruco.drawMarker(aruco_dict,i, 700)
        plt.imshow(image, cmap = mpl.cm.gray, interpolation = "nearest")
        grid.axis("off")

    plt.savefig("images/markers.pdf")
    plt.show()

def generate_charuco_board():
    calibrate_aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board = cv2.aruco.CharucoBoard_create(7,9,.025,.0125, calibrate_aruco_dict)
    img = board.draw((200*3,200*3))

    # save the calibration board
    cv2.imwrite('images/charuco.png',img)

if __name__ == "__main__":
    if len(sys.argv) > 1:
        if sys.argv[1] == "charuco":
            generate_charuco_board()
        elif sys.argv[1] == "aruco":
            generate_aruco_board()