#include <litevo/litevo.h>

int main() {
    litevo::Camera camera(500.0, 500.0, 320.0, 240.0, 640, 480);
    const litevo::Vec2 pixel = camera.Project(litevo::Vec3(0.0, 0.0, 1.0));

    return (pixel.x() == 320.0 && pixel.y() == 240.0 && litevo::kVersionMajor == 2) ? 0 : 1;
}
