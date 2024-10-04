import pygame
from pygame.locals import *
from gl import RendererRT
from figure import *
from material import *
from lights import *
from texture import Texture

width = 400
height = 320

screen = pygame.display.set_mode((width, height), pygame.SCALED)
clock = pygame.time.Clock()

rt = RendererRT(screen)
rt.envMap = Texture("c:/Users/DeLeon/Desktop/TR2/Raytracer2024/Raytracer2024/City.bmp")

# Materiales
texture1  = Material(texture=Texture("c:/Users/DeLeon/Desktop/TR2/Raytracer2024/Raytracer2024/Texture1.bmp"))
texture2  = Material(texture=Texture("c:/Users/DeLeon/Desktop/TR2/Raytracer2024/Raytracer2024/Texture2.bmp"), spec=128, ks=0.8, matType=REFLECTIVE)

Lock1  = Material(texture=Texture("c:/Users/DeLeon/Desktop/TR2/Raytracer2024/Raytracer2024/Lock1.bmp"))
Lock2  = Material(texture=Texture("c:/Users/DeLeon/Desktop/TR2/Raytracer2024/Raytracer2024/Lock2.bmp"), spec=128, ks=0.8, matType=REFLECTIVE)

mirror = Material(diffuse=[0.9, 0.9, 0.6], spec=128, ks=0.2, matType=REFLECTIVE)
blueMirror = Material(diffuse=[0.1, 0.9, 0.9], spec=128, ks=0.2, matType=REFLECTIVE)
brick = Material(diffuse=[0.2, 0.9, 0.2], spec=128, ks=0.2 )
grass = Material(diffuse=[0.9, 0.2, 0.9], spec=128, ks=0.2, )
white_material = Material(diffuse=[1, 1, 1], spec=128, ks=0.2)
# Luces
rt.light.append(DirectionalLight(direction=[-1, -1, -1], intensity=0.8))
rt.light.append(DirectionalLight(direction=[0.5, -0.5, -1], intensity=0.8, color=[1, 1, 1]))
rt.light.append(AmbientLight(intensity=0.1))

#cajas y disco
rt.scene.append(Disk(position = [0,1.5,-5], normal = [0,1,0], radius = 1.7, material = mirror))
rt.scene.append(AABB(position = [-1,-0.5,-7],sizes = [1,1,1], material = texture1))
rt.scene.append(AABB(position = [1,-0.5,-7],sizes = [1,1,1], material = Lock1))
rt.scene.append(Disk(position = [0,-1.5,-5], normal = [0,1,0], radius = 1.7, material = mirror))

#cuadros
rt.scene.append(Sphere(position=[-2.5, 1.5, -10], radius=1, material=Lock1))
rt.scene.append(Sphere(position=[2, 1.5, -10], radius=1, material=texture1))


# Añadir un plano cuadrado con un tamaño específico
rt.scene.append(SquarePlane(position=[-7.6, 0,5], normal=[70,0,10], size=8, material=Lock2))
rt.scene.append(SquarePlane(position=[7.6, 0,5], normal=[-70,0,10], size=8, material=Lock2))

rt.scene.append(SquarePlane(position=[0, 0, -4.5], normal=[0,0,10], size=1.6, material=brick))

rt.scene.append(Plane(position = [0,6,-5], normal = [1,0,0], material = blueMirror))
rt.scene.append(Plane(position = [0,-6,-5], normal = [0,-2,0], material = Lock2))



# Renderizar la escena
rt.glRender()

# Guardar la imagen como output.bmp
pygame.image.save(screen, "C:/Users/DeLeon/Documents/GitHub/RT3-Planes/Raytracer2024/Raytracer2024/output3.bmp")

isRunning = True
while isRunning:

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            isRunning = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                isRunning = False

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
