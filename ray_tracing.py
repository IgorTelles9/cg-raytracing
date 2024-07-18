import numpy as np
import cv2

class Vec3:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, v):
        return Vec3(self.x + v.x, self.y + v.y, self.z + v.z)

    def __sub__(self, v):
        return Vec3(self.x - v.x, self.y - v.y, self.z - v.z)

    def __mul__(self, v):
        if isinstance(v, Vec3):
            return Vec3(self.x * v.x, self.y * v.y, self.z * v.z)
        else:
            return Vec3(self.x * v, self.y * v, self.z * v)

    def __rmul__(self, v):
        return self.__mul__(v)

    def __truediv__(self, t):
        return Vec3(self.x / t, self.y / t, self.z / t)

    def dot(self, v):
        return self.x * v.x + self.y * v.y + self.z * v.z

    def normalize(self):
        mag = np.sqrt(self.x**2 + self.y**2 + self.z**2)
        return Vec3(self.x / mag, self.y / mag, self.z / mag)

class Ray:
    def __init__(self, origin, direction):
        self.origin = origin
        self.direction = direction.normalize()

def hit_sphere(center, radius, ray):
    oc = ray.origin - center
    a = ray.direction.dot(ray.direction)
    b = 2.0 * oc.dot(ray.direction)
    c = oc.dot(oc) - radius * radius
    discriminant = b * b - 4 * a * c
    if discriminant < 0:
        return False, None
    else:
        t = (-b - np.sqrt(discriminant)) / (2.0 * a)
        return True, t

def ray_color(ray, light_pos):
    sphere_center = Vec3(0, 0, -1)
    light_intensity = Vec3(1.5, 1.5, 1.5)
    ambient = 0.1

    hit, t = hit_sphere(sphere_center, 0.5, ray)
    if hit:
        hit_point = ray.origin + ray.direction * t
        N = (hit_point - sphere_center).normalize()
        light_dir = (light_pos - hit_point).normalize()
        diff = max(0.0, N.dot(light_dir))
        diff_color = Vec3(0, 0, 1) * diff
        ambient_color = Vec3(0, 0, 1) * ambient
        color = (ambient_color + diff_color) * light_intensity
        return Vec3(min(color.x, 1.0), min(color.y, 1.0), min(color.z, 1.0))

    t = 0.5 * (ray.direction.y + 1.0)
    return Vec3(1.0, 1.0, 1.0) * (1.0 - t) + Vec3(0.5, 0.7, 1.0) * t

def main():
    image_width = 800
    image_height = 400

    light_pos = Vec3(1, 1, 0)

    image = np.zeros((image_height, image_width, 3), dtype=np.uint8)

    origin = Vec3(0, 0, 0)
    horizontal = Vec3(4, 0, 0)
    vertical = Vec3(0, 2, 0)
    lower_left_corner = Vec3(-2, -1, -1)

    for j in range(image_height):
        for i in range(image_width):
            u = i / (image_width - 1)
            v = j / (image_height - 1)
            r = Ray(origin, lower_left_corner + horizontal * u + vertical * v - origin)
            color = ray_color(r, light_pos)
            image[image_height - j - 1, i] = [int(255.999 * min(max(color.x, 0.0), 1.0)),
                                              int(255.999 * min(max(color.y, 0.0), 1.0)),
                                              int(255.999 * min(max(color.z, 0.0), 1.0))]

    cv2.imwrite("output.png", image)

if __name__ == "__main__":
    main()
