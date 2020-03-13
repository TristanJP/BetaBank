import pygame
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *

import cv2
from PIL import Image
import numpy as np
from camera import Camera
from detection import Detection

import os
from cv2 import aruco
from frame_analyser import Frame_Analyser
from view import View

class Plane_Test():

    # constants
    INVERSE_MATRIX = np.array([[ 1.0, 1.0, 1.0, 1.0],
                               [-1.0,-1.0,-1.0,-1.0],
                               [-1.0,-1.0,-1.0,-1.0],
                               [ 1.0, 1.0, 1.0, 1.0]])

    plane_edges = (
        (0,1),
        (0,3),
        (2,1),
        (2,3)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
        )

    plane_verticies_central = (
        (1, -1, 0),
        (1, 1, 0),
        (-1, 1, 0),
        (-1, -1, 0)
        )

    texCoords = (
        (1.0, 0.0),
        (1.0, 1.0),
        (0.0, 1.0),
        (0.0, 0.0)
        )

    def __init__(self):
        # initialise webcam and start thread
        self.cam = Camera()
        self.cam.start()
 
        self.detection = Detection(self.cam.get_calibration_data())

    def Plane(self, struct=False):
        if struct:
            glBegin(GL_LINES)
            for edge in self.plane_edges:
                for vertex in edge:
                    glVertex3fv(plane_verticies_central[vertex])
            glEnd()
        else:
            i = 0
            glBegin(GL_QUADS)
            while i < len(self.plane_verticies_central):
                tex = self.texCoords[i]
                vert = self.plane_verticies_central[i]
                glTexCoord2f(tex[0], tex[1])
                glVertex3f(vert[0], vert[1], vert[2])
                i+=1
            glEnd()

    def loadTexture(self, frame):
        #textureSurface = pygame.image.load('images/test_text.jpg')
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = np.rot90(frame)
        frame = pygame.surfarray.make_surface(frame)
        textureSurface = frame
        textureData = pygame.image.tostring(textureSurface, "RGBA", 1)
        width = textureSurface.get_width()
        height = textureSurface.get_height()

        glEnable(GL_TEXTURE_2D)
        texid = glGenTextures(1)

        glBindTexture(GL_TEXTURE_2D, texid)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height,
                    0, GL_RGBA, GL_UNSIGNED_BYTE, textureData)

        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        return texid

    def main(self):
        pygame.init()
        display = (800,600)
        pygame.display.set_mode(display, DOUBLEBUF|OPENGL)


        gluPerspective(60, (display[0]/display[1]), 0.1, 90.0)

        glTranslatef(0.0,0.0, -5)

        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.cam.release_camera()
                    pygame.quit()
                    quit()

            glRotatef(1, 0, 1, 0)
            glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
            frame = self.cam.get_current_frame()
            self.loadTexture(frame)
            self.Plane()
            pygame.display.flip()
            pygame.time.wait(10)

if __name__ == "__main__":

    plane_test = Plane_Test()
    plane_test.main()