#include <cmath>
#include <iostream>
#include <vector>
#include "geometry.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION

#include "stb_image_write.h"

struct Sphere {
    Vec3f center;
    float radius;

    Sphere(const Vec3f &c, const float &r) : center(c), radius(r) {}

    bool ray_intersect(const Vec3f &orig, const Vec3f &dir, float &t0) const {
        Vec3f L = center - orig;
        float tca = L * dir;
        float d2 = L * L - tca * tca;
        if (d2 > radius * radius) return false;
        float thc = sqrtf(radius * radius - d2);
        t0 = tca - thc;
        float t1 = tca + thc;
        if (t0 < 0) t0 = t1;
        if (t0 < 0) return false;
        return true;
    }
};

Vec3f cast_ray(const Vec3f &orig, const Vec3f &dir, const Sphere &sphere) {
    float sphere_dist = std::numeric_limits<float>::max();
    if (!sphere.ray_intersect(orig, dir, sphere_dist)) {
        return Vec3f(0.2, 0.7, 0.8); // background color
    }

    return Vec3f(0.4, 0.4, 0.3);
}

template<typename T>
void render(const T object, char const *filename = "output.png") {
    const int width = 1024;
    const int height = 768;
    const int fov = M_PI / 2.;
    std::vector<Vec3f> framebuffer(width * height); // Frame buffer to hold pixel colors

    for (size_t j = 0; j < height; j++) {
        for (size_t i = 0; i < width; i++) {
            float x = (2 * (i + 0.5) / (float) width - 1) * tan(fov / 2.) * width / (float) height;
            float y = -(2 * (j + 0.5) / (float) height - 1) * tan(fov / 2.);
            Vec3f dir = Vec3f(x, y, -1).normalize();
            framebuffer[i + j * width] = cast_ray(Vec3f(0, 0, 0), dir, object);
        }
    }

    // 转换为 8-bit RGB 图像数据
    std::vector<unsigned char> image(3 * width * height);
    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            Vec3f &c = framebuffer[i + j * width];
            image[3 * (i + j * width) + 0] = (unsigned char) (255 * std::max(0.f, std::min(1.f, c.x)));
            image[3 * (i + j * width) + 1] = (unsigned char) (255 * std::max(0.f, std::min(1.f, c.y)));
            image[3 * (i + j * width) + 2] = (unsigned char) (255 * std::max(0.f, std::min(1.f, c.z)));
        }
    }

    // comp == 3 ==> RGB image
    if (stbi_write_png(filename, width, height, 3, image.data(), width * 3))
        std::cout << "successfully write image: " << filename << std::endl;
    else
        std::cout << "failed to write image!" << std::endl;
}

int main() {
    Sphere sphere(Vec3f(-3, 0, -16), 2);
    render<Sphere>(sphere);
    return 0;
}