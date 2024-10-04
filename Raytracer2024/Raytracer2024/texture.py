class Texture(object):
    def __init__(self, filename):
        with open(filename, "rb") as image:
            image.seek(10)
            headerSize = int.from_bytes(image.read(4), 'little')

            image.seek(18)
            self.width = int.from_bytes(image.read(4), 'little')
            self.height = int.from_bytes(image.read(4), 'little')

            image.seek(headerSize)

            self.pixels = []

            for y in range(self.height):
                pixelRow = []

                for x in range(self.width):
                    b = image.read(1)[0] / 255
                    g = image.read(1)[0] / 255
                    r = image.read(1)[0] / 255
                    pixelRow.append([r, g, b])

                self.pixels.append(pixelRow)

    def getColor(self, u, v):
        if 0 <= u < 1 and 0 <= v < 1:
            return self.pixels[int(v * self.height)][int(u * self.width)]
        else:
            return None






