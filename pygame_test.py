import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
#from screenDictionary import *

### Initialize variables ###
gameOn = True
dispWidth = 1920
dispHeight = 1024
bgImg = 'A1_64.png'
bgImg2 = 'HookedFront64.png'
transTuple = (255, 0, 255)
blackTuple = (100, 100, 100)
screenX = 0
screenY = 0

### Initialize display ###
def createDisplay(dispWidth, dispHeight):
    pygame.init()
    screen = pygame.display.set_mode((dispWidth, dispHeight), DOUBLEBUF|OPENGL)
    pygame.display.set_caption('Hooked')
    return screen

def createSurfaces(screen):
    background = pygame.Surface(screen.get_size())
    background.fill(blackTuple)
    sprite = pygame.Surface(screen.get_size())
    sprite.set_colorkey(transTuple)
    return background

def loadScene(bgImg):
    img = pygame.image.load(bgImg)
    textureData = pygame.image.tostring(img, "RGB", 1)
    width = img.get_width()
    height = img.get_height()
    bgImgGL = glGenTextures(1)
    glBindTexture(GL_TEXTURE_2D, bgImgGL)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, textureData)
    glEnable(GL_TEXTURE_2D)

def placeScene():
    glLoadIdentity()
    gluPerspective(90, 1, 0.05, 100)
    glTranslatef(0,0,0)
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
    glBegin(GL_QUADS)
    glTexCoord2f(0,0)
    glVertex3f(-4,-4,-4)
    glTexCoord2f(0,1)
    glVertex3f(-4,4,-4)
    glTexCoord2f(1,1)
    glVertex3f(4,4,-4)
    glTexCoord2f(1,0)
    glVertex3f(4,-4,-4)
    glEnd()    

screen = createDisplay(dispWidth, dispHeight)  
background = createSurfaces(screen)
mapParams = screenDictionary(screenX, screenY)
bgImg = mapParams[0]
loadScene(bgImg)
loadScene(bgImg2)
loadScene(bgImg)

###Run the game###
while gameOn:
    placeScene()
    pygame.display.flip()
    pygame.time.wait(1)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            gameOn = False