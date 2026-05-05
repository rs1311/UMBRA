#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#define PI 3.14159265358979323846

struct Vec3 {
    double x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    Vec3 operator-() const { return Vec3(-x, -y, -z); }
    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator*(double s) const { return Vec3(x * s, y * s, z * s); }
    Vec3 operator/(double s) const { return Vec3(x / s, y / s, z / s); }
    Vec3 operator*(const Vec3& v) const { return Vec3(x * v.x, y * v.y, z * v.z); }
    Vec3& operator+=(const Vec3& v) {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }
    double dot(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }
    Vec3 cross(const Vec3& v) const {
        return Vec3(
            y * v.z - z * v.y,
            z * v.x - x * v.z,
            x * v.y - y * v.x
        );
    }
    double length() const { return std::sqrt(x * x + y * y + z * z); }
    double length_squared() const { return x * x + y * y + z * z; }
    Vec3 normalize() const {
        double mag = length();
        if (mag < 1e-12) return Vec3(0, 0, 0);
        return *this / mag;
    }
};

inline Vec3 operator*(double s, const Vec3& v) {
    return v * s;
}

inline double clamp(double x, double minv, double maxv) {
    return std::max(minv, std::min(x, maxv));
}

inline Vec3 reflect(const Vec3& v, const Vec3& n) {
    return v - n * (2.0 * v.dot(n));
}

struct Ray {
    Vec3 origin;
    Vec3 direction;

    Ray() {}
    Ray(const Vec3& o, const Vec3& d) : origin(o), direction(d.normalize()) {}

    Vec3 at(double t) const {
        return origin + direction * t;
    }
};

struct Material {
    Vec3 albedo;
    double ambient;
    double diffuse;
    double specular;
    double shininess;
    double reflectivity;

    Material()
        : albedo(1.0, 1.0, 1.0), ambient(0.08), diffuse(1.0),
          specular(0.35), shininess(50.0), reflectivity(0.0) {}

    Material(const Vec3& a, double amb, double diff, double spec, double shiny, double refl)
        : albedo(a), ambient(amb), diffuse(diff), specular(spec),
          shininess(shiny), reflectivity(refl) {}
};

struct HitRecord {
    double t;
    Vec3 point;
    Vec3 normal;
    Material material;
    bool front_face;

    void set_face_normal(const Ray& ray, const Vec3& outward_normal) {
        front_face = ray.direction.dot(outward_normal) < 0;
        normal = front_face ? outward_normal : -outward_normal;
    }
};

struct Light {
    Vec3 position;
    Vec3 color;
    double intensity;

    Light(const Vec3& p, const Vec3& c, double i) : position(p), color(c), intensity(i) {}
};

class Hittable {
public:
    Material material;
    virtual ~Hittable() = default;
    virtual bool hit(const Ray& ray, double t_min, double t_max, HitRecord& rec) const = 0;
};

class Sphere : public Hittable {
public:
    Vec3 center;
    double radius;

    Sphere(const Vec3& c, double r, const Material& m) : center(c), radius(r) {
        material = m;
    }

    bool hit(const Ray& ray, double t_min, double t_max, HitRecord& rec) const override {
        Vec3 oc = ray.origin - center;
        double a = ray.direction.dot(ray.direction);
        double half_b = oc.dot(ray.direction);
        double c = oc.dot(oc) - radius * radius;
        double discriminant = half_b * half_b - a * c;

        if (discriminant < 0) return false;
        double sqrtd = std::sqrt(discriminant);

        double root = (-half_b - sqrtd) / a;
        if (root < t_min || root > t_max) {
            root = (-half_b + sqrtd) / a;
            if (root < t_min || root > t_max) return false;
        }

        rec.t = root;
        rec.point = ray.at(rec.t);
        Vec3 outward_normal = (rec.point - center) / radius;
        rec.set_face_normal(ray, outward_normal);
        rec.material = material;
        return true;
    }
};

class Plane : public Hittable {
public:
    Vec3 point;
    Vec3 normal;
    bool checkerboard;
    Vec3 alt_color;

    Plane(const Vec3& p, const Vec3& n, const Material& m, bool checker = false, const Vec3& alt = Vec3(0.12, 0.12, 0.12))
        : point(p), normal(n.normalize()), checkerboard(checker), alt_color(alt) {
        material = m;
    }

    bool hit(const Ray& ray, double t_min, double t_max, HitRecord& rec) const override {
        double denom = normal.dot(ray.direction);
        if (std::fabs(denom) < 1e-6) return false;

        double t = (point - ray.origin).dot(normal) / denom;
        if (t < t_min || t > t_max) return false;

        rec.t = t;
        rec.point = ray.at(t);
        rec.set_face_normal(ray, normal);
        rec.material = material;

        if (checkerboard) {
            int gx = static_cast<int>(std::floor(rec.point.x));
            int gz = static_cast<int>(std::floor(rec.point.z));
            bool checker = ((gx + gz) % 2) == 0;
            rec.material.albedo = checker ? material.albedo : alt_color;
        }

        return true;
    }
};

class Box : public Hittable {
public:
    Vec3 min_corner;
    Vec3 max_corner;
    Box(const Vec3& minc, const Vec3& maxc, const Material& m) : min_corner(minc), max_corner(maxc) { material = m;}
    bool hit(const Ray& ray, double t_min, double t_max, HitRecord& rec) const override {
        double tx0 = (min_corner.x - ray.origin.x) / ray.direction.x;
        double tx1 = (max_corner.x - ray.origin.x) / ray.direction.x;
        if (ray.direction.x < 0.0) std::swap(tx0, tx1);
        double ty0 = (min_corner.y - ray.origin.y) / ray.direction.y;
        double ty1 = (max_corner.y - ray.origin.y) / ray.direction.y;
        if (ray.direction.y < 0.0) std::swap(ty0, ty1);
        double tz0 = (min_corner.z - ray.origin.z) / ray.direction.z;
        double tz1 = (max_corner.z - ray.origin.z) / ray.direction.z;
        if (ray.direction.z < 0.0) std::swap(tz0, tz1);
        double t_enter = std::max(std::max(tx0, ty0), std::max(tz0, t_min));
        double t_exit = std::min(std::min(tx1, ty1), std::min(tz1, t_max));
        if (t_exit <= t_enter) return false;
        rec.t = t_enter;
        rec.point = ray.at(rec.t);
        const double eps = 1e-5;
        Vec3 outward_normal(0, 0, 0);
    	if (std::fabs(rec.point.x - min_corner.x) < eps) outward_normal = Vec3(-1, 0, 0);
        else if (std::fabs(rec.point.x - max_corner.x) < eps) outward_normal = Vec3(1, 0, 0);
        else if (std::fabs(rec.point.y - min_corner.y) < eps) outward_normal = Vec3(0, -1, 0);
        else if (std::fabs(rec.point.y - max_corner.y) < eps) outward_normal = Vec3(0, 1, 0);
        else if (std::fabs(rec.point.z - min_corner.z) < eps) outward_normal = Vec3(0, 0, -1);
        else outward_normal = Vec3(0, 0, 1);
        rec.set_face_normal(ray, outward_normal);
        rec.material = material;
        return true;
    }
};

class Scene {
public:
    std::vector<std::shared_ptr<Hittable>> objects;
    Light light = Light(Vec3(2.8, 4.5, 2.0), Vec3(1.0, 0.96, 0.92), 24.0);
    Vec3 sky_top = Vec3(0.50, 0.70, 1.00);
    Vec3 sky_bottom = Vec3(1.00, 1.00, 1.00);

    void add(const std::shared_ptr<Hittable>& object) {
        objects.push_back(object);
    }

    bool hit(const Ray& ray, double t_min, double t_max, HitRecord& rec) const {
        HitRecord temp;
        bool hit_anything = false;
        double closest = t_max;

        for (const auto& object : objects) {
            if (object->hit(ray, t_min, closest, temp)) {
                hit_anything = true;
                closest = temp.t;
                rec = temp;
            }
        }

        return hit_anything;
    }

    Vec3 background(const Ray& ray) const {
        Vec3 unit_dir = ray.direction.normalize();
        double t = 0.5 * (unit_dir.y + 1.0);
        return sky_bottom * (1.0 - t) + sky_top * t;
    }
};

class Camera {
public:
    Vec3 origin;
    Vec3 lookat;
    Vec3 world_up;
    double fov_deg;
    double aspect;

    Camera(const Vec3& o, const Vec3& l, double fov, double aspect_ratio)
        : origin(o), lookat(l), world_up(0, 1, 0), fov_deg(fov), aspect(aspect_ratio) {}

    Ray get_ray(double u, double v) const {
        Vec3 w = (origin - lookat).normalize();
        Vec3 uvec = world_up.cross(w).normalize();
        Vec3 vvec = w.cross(uvec).normalize();

        double viewport_height = 2.0 * std::tan((fov_deg * PI / 180.0) / 2.0);
        double viewport_width = aspect * viewport_height;

        Vec3 horizontal = uvec * viewport_width;
        Vec3 vertical = vvec * viewport_height;
        Vec3 lower_left_corner = origin - horizontal / 2.0 - vertical / 2.0 - w;

        Vec3 dir = lower_left_corner + horizontal * u + vertical * v - origin;
        return Ray(origin, dir);
    }
};

struct RNG {
    unsigned int state;

    explicit RNG(unsigned int seed = 123456789u) : state(seed) {}

    double next() {
        state = 1664525u * state + 1013904223u;
        return (state & 0x00FFFFFF) / double(0x01000000);
    }
};

Vec3 trace_ray(const Ray& ray, const Scene& scene, int depth) {
    if (depth <= 0) return Vec3(0, 0, 0);
    HitRecord rec;
    if (!scene.hit(ray, 0.001, std::numeric_limits<double>::infinity(), rec)) {
        return scene.background(ray);
    }
    Vec3 color = rec.material.albedo * rec.material.ambient;
    Vec3 view_dir = (-ray.direction).normalize();
    Vec3 to_light = scene.light.position - rec.point;
    double dist2 = to_light.length_squared();
    double dist = std::sqrt(dist2);
    Vec3 light_dir = to_light / dist;
    Ray shadow_ray(rec.point + rec.normal * 0.001, light_dir);
    HitRecord shadow_hit;
    bool occluded = scene.hit(shadow_ray, 0.001, dist - 0.002, shadow_hit);
    if (!occluded) {
        double attenuation = scene.light.intensity / dist2;
        double ndotl = std::max(0.0, rec.normal.dot(light_dir));
        Vec3 diffuse = rec.material.albedo * rec.material.diffuse * ndotl;

        Vec3 reflected_light = reflect(-light_dir, rec.normal).normalize();
        double spec_strength = std::pow(std::max(0.0, view_dir.dot(reflected_light)), rec.material.shininess);
        Vec3 specular = Vec3(1.0, 1.0, 1.0) * (rec.material.specular * spec_strength);

        color += (diffuse + specular) * scene.light.color * attenuation;
    }
    if (rec.material.reflectivity > 0.0) {
        Vec3 reflected_dir = reflect(ray.direction, rec.normal).normalize();
        Ray reflected_ray(rec.point + rec.normal * 0.001, reflected_dir);
        Vec3 reflected_color = trace_ray(reflected_ray, scene, depth - 1);
        color = color * (1.0 - rec.material.reflectivity) + reflected_color * rec.material.reflectivity;
    }
    return color;
}

void write_ppm(const std::string& filename, const std::vector<Vec3>& framebuffer, int width, int height) {
    std::ofstream out(filename);
    out << "P3\n" << width << " " << height << "\n255\n";
    for (int j = height - 1; j >= 0; --j) {
        for (int i = 0; i < width; ++i) {
            const Vec3& c = framebuffer[j * width + i];
            double r = std::sqrt(clamp(c.x, 0.0, 1.0));
            double g = std::sqrt(clamp(c.y, 0.0, 1.0));
            double b = std::sqrt(clamp(c.z, 0.0, 1.0));
            int ir = static_cast<int>(255.999 * r);
            int ig = static_cast<int>(255.999 * g);
            int ib = static_cast<int>(255.999 * b);
            out << ir << ' ' << ig << ' ' << ib << '\n';
        }
    }
}
void render_frame(const Scene& scene, const Camera& camera, int frame, int width, int height, int spp, int max_depth) {
    std::vector<Vec3> framebuffer(width * height);
    RNG rng(1337u + static_cast<unsigned int>(frame * 97u));

    for (int j = 0; j < height; ++j) {
        for (int i = 0; i < width; ++i) {
            Vec3 pixel_color(0, 0, 0);

            for (int s = 0; s < spp; ++s) {
                double u = (i + rng.next()) / (width - 1.0);
                double v = (j + rng.next()) / (height - 1.0);
                Ray ray = camera.get_ray(u, v);
                pixel_color += trace_ray(ray, scene, max_depth);
            }
            framebuffer[j * width + i] = pixel_color / static_cast<double>(spp);
        }
        std::cout << "Scanlines remaining: " << (height - 1 - j) << "\r" << std::flush;
    }
    std::cout << "\n";
    std::ostringstream filename;
    filename << "frame_" << std::setw(3) << std::setfill('0') << frame << ".ppm";
    write_ppm(filename.str(), framebuffer, width, height);
}

Scene build_scene() {
    Scene scene;

    Material checker_floor(Vec3(0.82, 0.82, 0.82), 0.12, 0.90, 0.08, 12.0, 0.0);
    Material blue_glossy(Vec3(0.20, 0.60, 1.00), 0.06, 0.85, 0.55, 90.0, 0.25);
    Material red_glossy(Vec3(1.00, 0.28, 0.28), 0.06, 0.85, 0.40, 70.0, 0.15);
    Material green_matte(Vec3(0.25, 0.85, 0.45), 0.10, 0.95, 0.12, 18.0, 0.0);
    Material gold_glossy(Vec3(0.95, 0.78, 0.20), 0.06, 0.80, 0.65, 100.0, 0.35);

    scene.add(std::make_shared<Plane>(Vec3(0, -0.75, 0), Vec3(0, 1, 0), checker_floor, true));
    scene.add(std::make_shared<Sphere>(Vec3(-0.95, -0.05, -2.7), 0.70, blue_glossy));
    scene.add(std::make_shared<Box>(Vec3(0.35, -0.55, -2.55), Vec3(1.45, 0.55, -1.45), red_glossy));
    scene.add(std::make_shared<Sphere>(Vec3(1.95, -0.25, -3.85), 0.50, green_matte));
    scene.add(std::make_shared<Box>(Vec3(-2.15, -0.60, -4.25), Vec3(-1.35, 0.10, -3.45), gold_glossy));

    return scene;
}

Camera build_orbit_camera(int frame, int total_frames, double aspect) {
    double t = (2.0 * PI * frame) / total_frames;
    Vec3 lookat(0.1, -0.1, -2.8);
    Vec3 origin(std::cos(t) * 3.4, 0.85, std::sin(t) * 3.4 - 2.8);
    return Camera(origin, lookat, 52.0, aspect);
}

void print_banner() {
    std::cout << "\n";
    std::cout << "========================================================\n";
    std::cout << " UMBRA 2 - FOCUSED EDITION\n";
    std::cout << " Scene-based CPU ray tracer with shadows, reflections,\n";
    std::cout << " anti-aliasing, materials, proper box normals, and an\n";
    std::cout << " orbit camera for animation.\n";
    std::cout << "========================================================\n\n";
}

int main() {
    print_banner();

    int width = 800;
    int height = 450;
    int spp = 16;
    int max_depth = 4;
    int total_frames = 60;

    std::cout << "Resolution width height (e.g. 800 450): ";
    std::cin >> width >> height;

    std::cout << "Samples per pixel (recommended 8-32): ";
    std::cin >> spp;

    std::cout << "Reflection recursion depth (recommended 3-5): ";
    std::cin >> max_depth;

    std::cout << "Total frames: ";
    std::cin >> total_frames;

    if (width < 64) width = 64;
    if (height < 64) height = 64;
    if (spp < 1) spp = 1;
    if (max_depth < 1) max_depth = 1;
    if (total_frames < 1) total_frames = 1;

    Scene scene = build_scene();
    double aspect = static_cast<double>(width) / static_cast<double>(height);

    for (int i = 0; i < total_frames; ++i) {
        Camera camera = build_orbit_camera(i, total_frames, aspect);
        std::cout << "Rendering frame " << i + 1 << "/" << total_frames << "...\n";
        render_frame(scene, camera, i, width, height, spp, max_depth);
    }

    std::cout << "Creating MP4 video with FFmpeg...\n";
    int ffmpeg_status = std::system("ffmpeg -y -framerate 20 -i frame_%03d.ppm -vcodec libx264 -pix_fmt yuv420p spin.mp4");

    if (ffmpeg_status != 0) {
        std::cout << "FFmpeg was not found on PATH or video creation failed.\n";
        std::cout << "Your PPM frames were still rendered successfully.\n";
    } else {
        std::cout << "Video created: spin.mp4\n";
    }

#ifdef _WIN32
    std::system("start spin.mp4");
#endif

    return 0;
}
