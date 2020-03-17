from OpenGL.GL import *
from OpenGL.GLU import *
import cv2
from PIL import Image
import numpy as np
from camera import Camera
from detection import Detection
import pygame
from pygame.locals import *
 
class OpenGLGlyphs:
 
    # constants
    INVERSE_MATRIX = np.array([[ 1.0, 1.0, 1.0, 1.0],
                               [-1.0,-1.0,-1.0,-1.0],
                               [-1.0,-1.0,-1.0,-1.0],
                               [ 1.0, 1.0, 1.0, 1.0]])
  
    def __init__(self):
        # initialise webcam and start thread
        self.webcam = Camera()
        self.webcam.start()
 
        # textures
        self.texture_background = None
        self.texture_cube = None
 
    def _init_gl(self, Width, Height):
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glClearDepth(1.0)
        glDepthFunc(GL_LESS)
        glEnable(GL_DEPTH_TEST)
        glShadeModel(GL_SMOOTH)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(33.7, 1.3, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
         
        # enable textures
        glEnable(GL_TEXTURE_2D)
        self.texture_background = glGenTextures(1)
        self.texture_cube = glGenTextures(1)
 
        # create cube texture 
        image = Image.open("devil.jpg")
        ix = image.size[0]
        iy = image.size[1]
        image = image.tostring("raw", "RGBX", 0, -1)
 
        glBindTexture(GL_TEXTURE_2D, self.texture_cube)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTexImage2D(GL_TEXTURE_2D, 0, 3, ix, iy, 0, GL_RGBA, GL_UNSIGNED_BYTE, image)
 
    def _draw_scene(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
 
        # get image from webcam
        image = self.webcam.get_current_frame()
 
        # convert image to OpenGL texture format
        bg_image = cv2.flip(image, 0)
        bg_image = Image.fromarray(bg_image)     
        ix = bg_image.size[0]
        iy = bg_image.size[1]
        bg_image = bg_image.tobytes("raw", "BGRX", 0, -1)
  
        # create background texture
        glBindTexture(GL_TEXTURE_2D, self.texture_background)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTexImage2D(GL_TEXTURE_2D, 0, 3, ix, iy, 0, GL_RGBA, GL_UNSIGNED_BYTE, bg_image)
         
        # draw background
        glBindTexture(GL_TEXTURE_2D, self.texture_background)
        glPushMatrix()
        glTranslatef(0.0,0.0,-10.0)
        self._draw_background()
        glPopMatrix()
 
        # handle glyph
        image = self._handle_glyph(image)
  
    def _handle_glyph(self, image):
 
        # attempt to detect glyph
        rvecs = None
        tvecs = None

        detection = Detection(self.webcam.get_calibration_data())

 
        try:
            frame_data = detection.get_markers_in_frame(image)
            rvecs, tvecs = frame_data["rvecs"], frame_data["tvecs"]
        except Exception as ex: 
            print(ex)
 
        if rvecs == None or tvecs == None: 
            return
 
        # build view matrix
        rmtx = cv2.Rodrigues(rvecs)[0]
 
        view_matrix = np.array([[rmtx[0][0],rmtx[0][1],rmtx[0][2],tvecs[0]],
                                [rmtx[1][0],rmtx[1][1],rmtx[1][2],tvecs[1]],
                                [rmtx[2][0],rmtx[2][1],rmtx[2][2],tvecs[2]],
                                [0.0       ,0.0       ,0.0       ,1.0    ]])
 
        view_matrix = view_matrix * self.INVERSE_MATRIX
 
        view_matrix = np.transpose(view_matrix)
 
        # load view matrix and draw cube
        glBindTexture(GL_TEXTURE_2D, self.texture_cube)
        glPushMatrix()
        glLoadMatrixd(view_matrix)
        self._draw_cube()
        glPopMatrix()
 
    def _draw_cube(self):
        # draw cube
        glBegin(GL_QUADS)
 
        glTexCoord2f(0.0, 0.0); glVertex3f( 0.0,  0.0,  0.0)
        glTexCoord2f(1.0, 0.0); glVertex3f( 1.0,  0.0,  0.0)
        glTexCoord2f(1.0, 1.0); glVertex3f( 1.0,  1.0,  0.0)
        glTexCoord2f(0.0, 1.0); glVertex3f( 0.0,  1.0,  0.0)
 
        glTexCoord2f(1.0, 0.0); glVertex3f( 0.0,  0.0, -1.0)
        glTexCoord2f(1.0, 1.0); glVertex3f( 0.0,  1.0, -1.0)
        glTexCoord2f(0.0, 1.0); glVertex3f( 1.0,  1.0, -1.0)
        glTexCoord2f(0.0, 0.0); glVertex3f( 1.0,  0.0, -1.0)
 
        glTexCoord2f(0.0, 1.0); glVertex3f( 0.0,  1.0, -1.0)
        glTexCoord2f(0.0, 0.0); glVertex3f( 0.0,  1.0,  0.0)
        glTexCoord2f(1.0, 0.0); glVertex3f( 1.0,  1.0,  0.0)
        glTexCoord2f(1.0, 1.0); glVertex3f( 1.0,  1.0, -1.0)
 
        glTexCoord2f(1.0, 1.0); glVertex3f( 0.0,  0.0, -1.0)
        glTexCoord2f(0.0, 1.0); glVertex3f( 1.0,  0.0, -1.0)
        glTexCoord2f(0.0, 0.0); glVertex3f( 1.0,  0.0,  0.0)
        glTexCoord2f(1.0, 0.0); glVertex3f( 0.0,  0.0,  0.0)
 
        glTexCoord2f(1.0, 0.0); glVertex3f( 1.0,  0.0, -1.0)
        glTexCoord2f(1.0, 1.0); glVertex3f( 1.0,  1.0, -1.0)
        glTexCoord2f(0.0, 1.0); glVertex3f( 1.0,  1.0,  0.0)
        glTexCoord2f(0.0, 0.0); glVertex3f( 1.0,  0.0,  0.0)
 
        glTexCoord2f(0.0, 0.0); glVertex3f( 0.0,  0.0, -1.0)
        glTexCoord2f(1.0, 0.0); glVertex3f( 0.0,  0.0,  0.0)
        glTexCoord2f(1.0, 1.0); glVertex3f( 0.0,  1.0,  0.0)
        glTexCoord2f(0.0, 1.0); glVertex3f( 0.0,  1.0, -1.0)
 
        glEnd()
 
    def _draw_background(self):
        # draw background
        glBegin(GL_QUADS)
        glTexCoord2f(0.0, 1.0); glVertex3f(-4.0, -3.0, 0.0)
        glTexCoord2f(1.0, 1.0); glVertex3f( 4.0, -3.0, 0.0)
        glTexCoord2f(1.0, 0.0); glVertex3f( 4.0,  3.0, 0.0)
        glTexCoord2f(0.0, 0.0); glVertex3f(-4.0,  3.0, 0.0)
        glEnd( )
 
    def main(self):
        pygame.init()
        display = (800,600)
        pygame.display.set_mode(display, DOUBLEBUF|OPENGL)

        gluPerspective(60, (display[0]/display[1]), 0.1, 90.0)

        glTranslatef(0.0,0.0, -5)

        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    quit()

            glRotatef(1, 0, 1, 0)
            glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
            self._draw_scene()
            #Cube()
            pygame.display.flip()
            pygame.time.wait(10)

if __name__ == "__main__":

    openGLGlyphs = OpenGLGlyphs()
    openGLGlyphs.main()
