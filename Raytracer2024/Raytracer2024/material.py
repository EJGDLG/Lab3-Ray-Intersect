from MathLib import reflectVector
from lights import *
OPAQUE = 0
REFLECTIVE = 1
TRANSPARENT = 2

class Material(object):
    def __init__(self, diffuse=[1, 1, 1], spec=1.0, ks=0.0, texture=None, matType=OPAQUE, opacity=1.0):
        self.diffuse = diffuse
        self.spec = spec
        self.ks = ks
        self.texture = texture
        self.matType = matType
        self.opacity = opacity 

    def GetSurfaceColor(self, intercept, renderer, recursion=0):
        lightColor = [0, 0, 0]
        reflectColor = [0, 0, 0]
        finalColor = self.diffuse

        if self.texture and intercept.texCoords:
            textureColor = self.texture.getColor(intercept.texCoords[0], intercept.texCoords[1])
            finalColor = [finalColor[i] * textureColor[i] for i in range(3)]

        for light in renderer.light:
            shadowIntercept = None

            if light.lightType == "Directional":
                lightDir = [-i for i in light.direction]
                shadowIntercept = renderer.glCastRay(intercept.point, lightDir, intercept.obj)

            if shadowIntercept is None:
                lightColor = [(lightColor[i] + light.GetSpecularColor(intercept, renderer.camera.translate)[i]) for i in range(3)]

                if self.matType == OPAQUE:
                    lightColor = [(lightColor[i] + light.GetLightColor(intercept)[i]) for i in range(3)]
                else:
                    lightColor = renderer.clearColor  # For transparency, use clear color

        if self.matType == REFLECTIVE or self.matType == TRANSPARENT:
            rayDir = [-i for i in intercept.rayDirection]
            reflect = reflectVector(intercept.normal, rayDir)

            # Reverse the direction for transparent materials
            if self.matType == TRANSPARENT:
                reflect = [-i for i in reflect]

            reflectIntercept = renderer.glCastRay(intercept.point, reflect, intercept.obj, recursion + 1)
            if reflectIntercept is not None:
                reflectColor = reflectIntercept.obj.material.GetSurfaceColor(reflectIntercept, renderer, recursion + 1)
            else:
                reflectColor = renderer.glEnvMapColor(intercept.point, reflect)

        finalColor = [(finalColor[i] * (lightColor[i] + reflectColor[i])) for i in range(3)]
        finalColor = [min(1, finalColor[i]) for i in range(3)]
        return finalColor
