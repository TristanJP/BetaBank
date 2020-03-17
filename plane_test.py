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

        # initialise texture
        self.texture_background = None

    def Plane(self, scale, struct=False):
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
                glVertex3f(vert[0]*scale, vert[1]*scale, vert[2]*scale)
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

        texid = glGenTextures(1)

        glBindTexture(GL_TEXTURE_2D, texid)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height,
                    0, GL_RGBA, GL_UNSIGNED_BYTE, textureData)

        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        return texid

    def draw_background(self, frame):
        # convert image to OpenGL texture format
        
        bg_image = cv2.flip(frame, 0)
        bg_image = Image.fromarray(bg_image)     
        ix = bg_image.size[0]
        iy = bg_image.size[1]
        bg_image = bg_image.tobytes('raw', 'BGRX', 0, -1)

        # create background texture
        glBindTexture(GL_TEXTURE_2D, self.texture_background)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTexImage2D(GL_TEXTURE_2D, 0, 3, ix, iy, 0, GL_RGBA, GL_UNSIGNED_BYTE, bg_image)
        
        # draw background
        glBindTexture(GL_TEXTURE_2D, self.texture_background)
        glPushMatrix()
        #self.loadTexture(frame)
        #glTranslatef(0.0,0.0,-10.0)
        glBegin(GL_QUADS)
        glTexCoord2f(0.0, 1.0); glVertex3f(-4.0, -3.0, 0.0)
        glTexCoord2f(1.0, 1.0); glVertex3f( 4.0, -3.0, 0.0)
        glTexCoord2f(1.0, 0.0); glVertex3f( 4.0,  3.0, 0.0)
        glTexCoord2f(0.0, 0.0); glVertex3f(-4.0,  3.0, 0.0)
        glEnd( )
        glPopMatrix()

        return bg_image

    def main(self):
        pygame.init()
        width = 1280
        height = 720
        display = (width, height)
        pygame.display.set_mode(display, DOUBLEBUF|OPENGL)

        glEnable(GL_TEXTURE_2D)

        image = cv2.imread("images/test_text.jpg")

        gluPerspective(60, (display[0]/display[1]), 0.1, 90.0)
        glTranslatef(0.0,0.0, -5)

        self.texture_background = glGenTextures(1)
        
        #glRotatef(1, 0, 1, 0)

        search_aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.cam.release_camera()
                    pygame.quit()
                    quit()
            
            glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
            glLoadIdentity()

            gluPerspective(60, (display[0]/display[1]), 0.1, 90.0)
            glTranslatef(0.0,0.0, -5)
            #glRotatef(45, 0, 1, 0)

            frame = self.cam.get_current_frame()
            
            self.draw_background(frame)
            
            frame_data = self.detection.get_markers_in_frame(frame, search_aruco_dict)

            if 1 in frame_data["ids"]:
                tvecs = frame_data["ids"][1]["marker_tvecs"][0]
                x = tvecs[0]
                y = tvecs[1]
                z = tvecs[2]

                #glTranslatef(x,y, z)
                #self.draw_rect(0, 0, 200, 100)

            #self.refresh2d(width, height)

            #glColor3f(0.0, 0.0, 1.0)

            #glTranslatef(0.0,0.0, -5)
            
            #self.loadTexture(image)
            #self.Plane(0.5)
            #
            # glRotatef(1, 0, 1, 0)

            
            #glTranslatef(0.0,0.0, -0.5)


            pygame.display.flip()
            pygame.time.wait(10)


    def refresh2d(self, width, height):
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(0.0, width, 0.0, height, 0.0, 1.0)
        glMatrixMode (GL_MODELVIEW)
        glLoadIdentity()

    def draw_rect(self, x, y, width, height):
        glBegin(GL_QUADS)                                  # start drawing a rectangle
        glVertex2f(x, y)                                   # bottom left point
        glVertex2f(x + width, y)                           # bottom right point
        glVertex2f(x + width, y + height)                  # top right point
        glVertex2f(x, y + height)                          # top left point
        glEnd()

if __name__ == "__main__":

    plane_test = Plane_Test()
    plane_test.main()