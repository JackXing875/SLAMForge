#include <slamforge/slamforge.h>

int main() {
    slamforge::Camera camera(500.0, 500.0, 320.0, 240.0, 640, 480);
    const slamforge::Vec2 pixel = camera.Project(slamforge::Vec3(0.0, 0.0, 1.0));

    return (pixel.x() == 320.0 && pixel.y() == 240.0 && slamforge::kVersionMajor == 3) ? 0 : 1;
}
