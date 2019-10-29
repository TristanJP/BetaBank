import cv2
import numpy as np
from matplotlib import pyplot as plt

cap = cv2.VideoCapture(0)
cap.set(3, 1280)
cap.set(4, 720)

img = cv2.imread("thai.jpg")
alpha = 0.4

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    frame = cv2.flip(frame,1)

    added_image = cv2.addWeighted(frame[100:640,100:640,:],alpha,img[0:540,0:540,:],1-alpha,0)

    frame[100:640,100:640,:] = added_image

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    cv2.

    # Display the resulting frame
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()