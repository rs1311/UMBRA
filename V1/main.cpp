#include <fstream>
#include <iostream>
#include <cmath>
#include <iomanip>
#include <string>
#include <sstream>

#define PI 3.14159265358979323846

std::string shape_choice = "sphere"; // default, will be set by user

struct Vec3 {
    double x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    Vec3 operator+(const Vec3& v) const { return Vec3(x+v.x, y+v.y, z+v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x-v.x, y-v.y, z-v.z); }
    Vec3 operator*(double s) const { return Vec3(x*s, y*s, z*s); }
    Vec3 operator/(double s) const { return Vec3(x/s, y/s, z/s); }
    double dot(const Vec3& v) const { return x*v.x + y*v.y + z*v.z; }
    Vec3 cross(const Vec3& v) const {
        return Vec3(y * v.z - z * v.y,
                    z * v.x - x * v.z,
                    x * v.y - y * v.x);
    }
    Vec3 normalize() const {
        double mag = std::sqrt(x*x + y*y + z*z);
        return *this / mag;
    }
};

struct Ray {
    Vec3 origin, direction;
    Ray(const Vec3& o, const Vec3& d) : origin(o), direction(d) {}
};

bool hit_sphere(const Vec3& center, double radius, const Ray& ray, double& t) {
    Vec3 oc = ray.origin - center;
    double a = ray.direction.dot(ray.direction);
    double b = 2.0 * oc.dot(ray.direction);
    double c = oc.dot(oc) - radius * radius;
    double discriminant = b*b - 4*a*c;
    if (discriminant < 0) return false;
    t = (-b - std::sqrt(discriminant)) / (2.0 * a);
    return true;
}

struct AABB {
    Vec3 min, max;
};

bool hit_aabb(const AABB& box, const Ray& ray, double t_min, double t_max) {
    for (int i = 0; i < 3; ++i) {
        double origin_component = (i == 0 ? ray.origin.x : (i == 1 ? ray.origin.y : ray.origin.z));
        double direction_component = (i == 0 ? ray.direction.x : (i == 1 ? ray.direction.y : ray.direction.z));
        double min_component = (i == 0 ? box.min.x : (i == 1 ? box.min.y : box.min.z));
        double max_component = (i == 0 ? box.max.x : (i == 1 ? box.max.y : box.max.z));

        double invD = 1.0 / direction_component;
        double t0 = (min_component - origin_component) * invD;
        double t1 = (max_component - origin_component) * invD;
        if (invD < 0.0) std::swap(t0, t1);
        t_min = t0 > t_min ? t0 : t_min;
        t_max = t1 < t_max ? t1 : t_max;
        if (t_max <= t_min) return false;
    }
    return true;
}

bool hit_plane(const Vec3& point, const Vec3& normal, const Ray& ray, double& t) {
    double denom = normal.dot(ray.direction);
    if (std::fabs(denom) < 1e-6) return false;
    t = (point - ray.origin).dot(normal) / denom;
    return t >= 0.001;
}

Vec3 ray_color(const Ray& r) {
    double t;
    Vec3 light_dir = Vec3(1, 1, -1).normalize();

    if (shape_choice == "sphere") {
        Vec3 sphere_center(0, 0, -1);
        if (hit_sphere(sphere_center, 0.5, r, t)) {
            Vec3 N = (r.origin + r.direction * t - sphere_center).normalize();
            double brightness = std::max(0.0, N.dot(light_dir));
            return Vec3(0.2, 0.6, 1.0) * brightness;
        }
    } else if (shape_choice == "cube") {
        AABB cube = {Vec3(-0.5, -0.5, -1.5), Vec3(0.5, 0.5, -0.5)};
        if (hit_aabb(cube, r, 0.001, 1000.0)) {
            double brightness = std::max(0.0, Vec3(0, 0, 1).dot(light_dir));
            return Vec3(1.0, 0.3, 0.3) * brightness;
        }
    } else if (shape_choice == "plane") {
        Vec3 plane_point(0, -0.5, -1);   // point on the plane
        Vec3 plane_normal(0, 1, 0);      // upward facing
        if (hit_plane(plane_point, plane_normal, r, t)) {
            Vec3 hit_point = r.origin + r.direction * t;
            bool checker = ((int(std::floor(hit_point.x)) + int(std::floor(hit_point.z))) % 2) == 0;
            Vec3 base_color = checker ? Vec3(0.8, 0.8, 0.8) : Vec3(0.1, 0.1, 0.1);
            double brightness = std::max(0.0, plane_normal.dot(light_dir));
            return base_color * brightness;
        }
    }

    // Sky background
    Vec3 unit_dir = r.direction.normalize();
    t = 0.5 * (unit_dir.y + 1.0);
    return Vec3(1.0, 1.0, 1.0) * (1.0 - t) + Vec3(0.5, 0.7, 1.0) * t;
}

Ray get_ray(double u, double v, const Vec3& origin, const Vec3& lookat) {
    Vec3 w = (origin - lookat).normalize();
    Vec3 up(0, 1, 0);
    Vec3 uvec = up.cross(w).normalize();
    Vec3 vvec = w.cross(uvec);

    double fov = 90.0;
    double aspect = 2.0;
    double viewport_height = 2.0 * std::tan((fov * PI / 180.0) / 2.0);
    double viewport_width = aspect * viewport_height;

    Vec3 horizontal = uvec * viewport_width;
    Vec3 vertical = vvec * viewport_height;
    Vec3 lower_left_corner = origin - horizontal/2 - vertical/2 - w;

    Vec3 dir = lower_left_corner + horizontal * u + vertical * v - origin;
    return Ray(origin, dir);
}

void render_frame(int frame, double angle_deg, int width, int height) {
    double angle_rad = angle_deg * PI / 180.0;
    double radius = 2.0;

    Vec3 lookat(0, 0, -1);
    Vec3 origin(std::cos(angle_rad) * radius, 0.0, std::sin(angle_rad) * radius - 1.0);

    std::ostringstream filename;
    filename << "frame_" << std::setw(3) << std::setfill('0') << frame << ".ppm";
    std::ofstream out(filename.str());
    out << "P3\n" << width << " " << height << "\n255\n";

    for (int j = height - 1; j >= 0; --j) {
        for (int i = 0; i < width; ++i) {
            double u = double(i) / width;
            double v = double(j) / height;
            Ray r = get_ray(u, v, origin, lookat);
            Vec3 color = ray_color(r);
            int ir = int(255.99 * color.x);
            int ig = int(255.99 * color.y);
            int ib = int(255.99 * color.z);
            out << ir << " " << ig << " " << ib << "\n";
        }
    }
    out.close();
}

int main() {
	std::cout << "   _                  _   _         _               _           _          " << '\n';
	std::cout << "  /\\_\\               /\\_\\/\\_\\ _    / /\\            /\\ \\        / /\\        " << '\n';
	std::cout << " / / /         _    / / / / //\\_\\ / /  \\          /  \\ \\      / /  \\       " << '\n';
	std::cout << " \\ \\ \\__      /\\_\\ /\\ \\/ \\ \\/ / // / /\\ \\        / /\\ \\ \\    / / /\\ \\      " << '\n';
	std::cout << "  \\ \\___\\    / / //  \\____\\__/ // / /\\ \\ \\      / / /\\ \\_\\  / / /\\ \\ \\     " << '\n';
	std::cout << "   \\__  /   / / // /\\/________// / /\\ \\_\\ \\    / / /_/ / / / / /  \\ \\ \\    " << '\n';
	std::cout << "   / / /   / / // / /\\/_// / // / /\\ \\ \\___\\  / / /__\\/ / / / /___/ /\\ \\   " << '\n';
	std::cout << "  / / /   / / // / /    / / // / /  \\ \\ \\__/ / / /_____/ / / /_____/ /\\ \\  " << '\n';
	std::cout << " / / /___/ / // / /    / / // / /____\\_\\ \\  / / /\\ \\ \\  / /_________/\\ \\ \\ " << '\n';
	std::cout << "/ / /____\\/ / \\/_/    / / // / /__________\\/ / /  \\ \\ \\/ / /_       __\\ \\_\\" << '\n';
	std::cout << "\\/_________/          \\/_/ \\/_____________/\\/_/    \\_\\/\\_\\___\\     /____/_/" << '\n';
    
	std::cout << "Welcome to Umbra! \n Umbra is a open-source ray tracing renderer that can render a 3D pan of a select few objects. This was developed by me, Raghav Sriram, in order to gain a deeper understanding of computer graphics and C++ programming. I hope you enjoy!\n\n\n";	
	std::cout << "Menu Options : \n(1)Instructions \n(2)Rendering Engine \n(3)Video Issues \nOption: ";
	int menuchoice =2;
	std::cin >>menuchoice;
	if (menuchoice==1){
		std::cout<<"\n\nHere, you will be prompted to enter a 3D object name. Once you enter the name, the renderer will load the snapshots of the object with light rays hitting it and save them as ppm files. Then, the renderer uses ffmpeg to piece everything together into a MP4 file. The video is autolaunched, and shows a pan of the object.";
	}
	else if(menuchoice==2){
		std::cout << "\n\nChoose shape to render (sphere/cube/plane): ";
	    std::cin >> shape_choice;
	    if (shape_choice != "sphere" && shape_choice != "cube" && shape_choice != "plane") {
	        std::cout<<"\n\n You have entered an invalid option. Restarting renderer.\n\n\n";
			main();
	    }
	
	    const int width = 400;
	    const int height = 200;
	    const int total_frames = 60;
	
	    for (int i = 0; i < total_frames; ++i) {
	        double angle = (360.0 / total_frames) * i;
	        std::cout << "Rendering frame " << i << " at angle " << angle << " degrees... ("<<round((i/60.0)*100.0)<<"%)\n";
	        render_frame(i, angle, width, height);
	    }
	
	    std::cout << "Creating MP4 video...\n";
	    system("\"C:\\ffmpeg\\bin\\ffmpeg.exe\" -y -framerate 20 -i frame_%03d.ppm -vcodec libx264 -pix_fmt yuv420p spin.mp4");
	    system("start spin.mp4"); // Opens it with the default video player on Windows
	    return 0;
	}
	else if(menuchoice==3){
		std::cout<<"\n\n If you are experiencing problems with the video loading, video creation or video launching, please install the 7z file for ffmpeg, extract it, move it to a folder on your PC, add a windows path variable with the address as the parameter, and verify that it works by interacting with ffmpeg through the command prompt. If this still does not work, consider restarting the file or reinstalling ffmpeg. Keep in mind, if you want to copy over the commands from the C++ source code here, there are escape sequences in the command text that you need to get rid of. Thank you!";
	}
	else{
		std::cout<<"\n\n You have entered an invalid option. Restarting renderer.\n\n\n";
		main();
	}
}
