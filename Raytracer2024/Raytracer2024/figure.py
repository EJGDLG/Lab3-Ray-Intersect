from intercep import Intercep
from math import atan2, acos, pi, isclose, sqrt

def vector_subtract(v1, v2):
    return [v1[i] - v2[i] for i in range(len(v1))]

def vector_add(v1, v2):
    return [v1[i] + v2[i] for i in range(len(v1))]

def vector_multiply(v, scalar):
    return [v[i] * scalar for i in range(len(v))]

def vector_dot(v1, v2):
    return sum(v1[i] * v2[i] for i in range(len(v1)))

def vector_norm(v):
    return sqrt(sum(v[i] ** 2 for i in range(len(v))))

def normalize(v):
    norm = vector_norm(v)
    return [v[i] / norm for i in range(len(v))]

class Shape(object):
    def __init__(self, position, material):
        self.position = position
        self.material = material
        self.type = "None"
    
    def ray_intersect(self, orig, dir):
        return None
    

class Sphere(Shape):
    def __init__(self, position, radius, material):
        super().__init__(position, material)
        self.radius = radius
        self.type = "Sphere"
        
    def ray_intersect(self, orig, dir):
        L = vector_subtract(self.position, orig)
        tca = vector_dot(L, dir)
        d = sqrt(vector_dot(L, L) - tca ** 2)
        
        if d > self.radius:
            return None
        
        thc = sqrt(self.radius ** 2 - d ** 2)
        t0 = tca - thc
        t1 = tca + thc

        if t0 < 0:
            t0 = t1
        if t0 < 0:
            return None
        
        P = vector_add(orig, vector_multiply(dir, t0))
        normal = vector_subtract(P, self.position)
        normal = normalize(normal)

        u = (atan2(normal[2], normal[0])) / (2 * pi) + 0.5
        v = acos(-normal[1]) / pi

        return Intercep(point=P, normal=normal, distance=t0, texCoords=[u, v], rayDirection=dir, obj=self)

class Plane(Shape):
    def __init__(self, position, normal, material):
        super().__init__(position, material)
        self.normal = normalize(normal)
        self.type = "Plane"

    def ray_intersect(self, orig, dir):
        denom = vector_dot(dir, self.normal)
        if isclose(denom, 0):
            return None
        num = vector_dot(vector_subtract(self.position, orig), self.normal)
        t = num / denom

        if t < 0:
            return None

        P = vector_add(orig, vector_multiply(dir, t))
        return Intercep(point=P, normal=self.normal, distance=t, texCoords=None, rayDirection=dir, obj=self)

class Disk(Plane):
    def __init__(self, position, normal, radius, material):
        super().__init__(position, normal, material)
        self.radius = radius
        self.type = "Disk"

    def ray_intersect(self, orig, dir):
        plane_intercept = super().ray_intersect(orig, dir)
        if plane_intercept is None:
            return None

        contact = vector_subtract(plane_intercept.point, self.position)
        contact = vector_norm(contact)

        if contact > self.radius:
            return None
        return plane_intercept

class SquarePlane(Plane):
    def __init__(self, position, normal, size, material):
        super().__init__(position, normal, material)
        self.size = size
        self.type = "SquarePlane"

    def ray_intersect(self, orig, dir):
        plane_intercept = super().ray_intersect(orig, dir)
        if plane_intercept is None:
            return None

        contact = vector_subtract(plane_intercept.point, self.position)
        if abs(contact[0]) > self.size / 2 or abs(contact[1]) > self.size / 2:
            return None

        return plane_intercept

class AABB(Shape):
    def __init__(self, position, sizes, material):
        super().__init__(position, material)
        self.sizes = sizes
        self.type = "AABB"
        self.planes = []

        right_plane = Plane([position[0] + sizes[0] / 2, position[1], position[2]], [1, 0, 0], material)
        left_plane = Plane([position[0] - sizes[0] / 2, position[1], position[2]], [-1, 0, 0], material)
        up_plane = Plane([position[0], position[1] + sizes[1] / 2, position[2]], [0, 1, 0], material)
        down_plane = Plane([position[0], position[1] - sizes[1] / 2, position[2]], [0, -1, 0], material)
        front_plane = Plane([position[0], position[1], position[2] + sizes[2] / 2], [0, 0, 1], material)
        back_plane = Plane([position[0], position[1], position[2] - sizes[2] / 2], [0, 0, -1], material)

        self.planes.extend([right_plane, left_plane, up_plane, down_plane, front_plane, back_plane])

        epsilon = 0.001
        self.bounds_min = [position[i] - (epsilon + sizes[i] / 2) for i in range(3)]
        self.bounds_max = [position[i] + (epsilon + sizes[i] / 2) for i in range(3)]

    def ray_intersect(self, orig, dir):
        intercept = None
        t = float("inf")

        for plane in self.planes:
            plane_intercept = plane.ray_intersect(orig, dir)
            if plane_intercept is not None:
                plane_point = plane_intercept.point
                if all(self.bounds_min[i] <= plane_point[i] <= self.bounds_max[i] for i in range(3)):
                    if plane_intercept.distance < t:
                        t = plane_intercept.distance
                        intercept = plane_intercept

        if intercept is None:
            return None

        u, v = 0, 0
        if abs(intercept.normal[0]) > 0:
            u = (intercept.point[1] - self.bounds_min[1]) / self.sizes[1]
            v = (intercept.point[2] - self.bounds_min[2]) / self.sizes[2]
        elif abs(intercept.normal[1]) > 0:
            u = (intercept.point[0] - self.bounds_min[0]) / self.sizes[0]
            v = (intercept.point[2] - self.bounds_min[2]) / self.sizes[2]
        elif abs(intercept.normal[2]) > 0:
            u = (intercept.point[0] - self.bounds_min[0]) / self.sizes[0]
            v = (intercept.point[1] - self.bounds_min[1]) / self.sizes[1]

        u = min(0.999, max(0, u))
        v = min(0.999, max(0, v))

        return Intercep(point=intercept.point, normal=intercept.normal, distance=t, texCoords=[u, v], rayDirection=dir, obj=self)
class Triangle(Shape):
    def __init__(self, v0, v1, v2, material):
        super().__init__(position=None, material=material)
        self.v0 = v0
        self.v1 = v1
        self.v2 = v2
        self.normal = normalize(vector_cross(vector_subtract(v1, v0), vector_subtract(v2, v0)))
        self.type = "Triangle"
    
    def ray_intersect(self, orig, dir):
        edge1 = vector_subtract(self.v1, self.v0)
        edge2 = vector_subtract(self.v2, self.v0)
        h = vector_cross(dir, edge2)
        a = vector_dot(edge1, h)

        if isclose(a, 0):
            return None
        
        f = 1.0 / a
        s = vector_subtract(orig, self.v0)
        u = f * vector_dot(s, h)
        
        if u < 0.0 or u > 1.0:
            return None
        
        q = vector_cross(s, edge1)
        v = f * vector_dot(dir, q)
        
        if v < 0.0 or u + v > 1.0:
            return None
        
        t = f * vector_dot(edge2, q)
        if t > 0.0001:
            P = vector_add(orig, vector_multiply(dir, t))
            return Intercep(point=P, normal=self.normal, distance=t, texCoords=[u, v], rayDirection=dir, obj=self)
        else:
            return None
class OBB(Shape):
    def __init__(self, position, sizes, rotation_matrix, material):
        super().__init__(position, material)
        self.sizes = sizes
        self.rotation_matrix = rotation_matrix 
        self.type = "OBB"
    
    def ray_intersect(self, orig, dir):
        inv_rotation = inverse_matrix(self.rotation_matrix)
        transformed_orig = matrix_multiply(inv_rotation, vector_subtract(orig, self.position))
        transformed_dir = matrix_multiply(inv_rotation, dir)

        aabb = AABB([0, 0, 0], self.sizes, self.material)
        intercept = aabb.ray_intersect(transformed_orig, transformed_dir)
        
        if intercept is not None:
            intercept.point = matrix_multiply(self.rotation_matrix, intercept.point)
            intercept.point = vector_add(intercept.point, self.position)
        return intercept
class Cylinder(Shape):
    def __init__(self, position, radius, height, material):
        super().__init__(position, material)
        self.radius = radius
        self.height = height
        self.type = "Cylinder"
    
    def ray_intersect(self, orig, dir):
    
        pass
class Capsule(Shape):
    def __init__(self, position, radius, height, material):
        super().__init__(position, material)
        self.radius = radius
        self.height = height
        self.type = "Capsule"
    
    def ray_intersect(self, orig, dir):
        pass
class Torus(Shape):
    def __init__(self, position, ring_radius, tube_radius, material):
        super().__init__(position, material)
        self.ring_radius = ring_radius
        self.tube_radius = tube_radius
        self.type = "Torus"
    
    def ray_intersect(self, orig, dir):
        pass
class Ellipsoid(Shape):
    def __init__(self, position, radii, material):
        super().__init__(position, material)
        self.radii = radii  # [rx, ry, rz]
        self.type = "Ellipsoid"
    
    def ray_intersect(self, orig, dir):
        pass
