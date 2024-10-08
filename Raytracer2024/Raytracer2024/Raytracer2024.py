import pygame
from pygame.locals import *
from gl import RendererRT
from figure import *
from material import *
from lights import *
from texture import Texture

width = 1200
height = 620

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
blueMirror = Material(diffuse=[0.1, 0.9, 0.9], spec=128, ks=0.2, matType=TRANSPARENT)
brick = Material(diffuse=[0.2, 0.9, 0.2], spec=128, ks=0.2 )
grass = Material(diffuse=[0.9, 0.2, 0.9], spec=128, ks=0.2, )
white_material = Material(diffuse=[1, 1, 1], spec=128, ks=0.2)
# Luces
rt.light.append(DirectionalLight(direction=[-1, -1, -1], intensity=0.8))
rt.light.append(DirectionalLight(direction=[0.5, -0.5, -1], intensity=0.8, color=[1, 1, 1]))
rt.light.append(AmbientLight(intensity=0.1))

rt.scene.append(Cylinder(position=[0, -2.5, -5], radius=0.5, height=2, material=mirror))

# Añadir elipsoides
rt.scene.append(Ellipsoid(position=[0, 0.5, -12], radii=[2, 1.5, 1.5], material=blueMirror))

# Añadir cilindros
rt.scene.append(Cylinder(position=[-4, -1, -5], radius=0.5, height=2, material=texture1))
rt.scene.append(Cylinder(position=[4, -1, -5], radius=0.5, height=2, material=blueMirror))

# Añadir triángulos

rt.scene.append(Triangle(A=[-7, 5, -10], B=[-5, 2, -10], C=[-7, 2, -6], material=blueMirror))
rt.scene.append(Triangle(A=[7, -5, -10], B=[5, -2, -10], C=[7, -2, -6], material=texture2))
rt.scene.append(Triangle(A=[-2.5, 3, -10], B=[2.5, 3, -10], C=[-1.5, 3, -6], material=grass))


# Renderizar la escena
rt.glRender()

# Guardar la imagen como output.bmp
pygame.image.save(screen, "C:/Users/DeLeon/Documents/GitHub/Lab3-Ray-Intersect/Raytracer2024/Raytracer2024/output4.bmp")

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
