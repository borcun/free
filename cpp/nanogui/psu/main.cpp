#include "psu.h"

/// window width
#define WIN_WIDTH  (240)
/// window height
#define WIN_HEIGHT (200)

int main(int argc, char **argv) {
    nanogui::init();

    new PSUScreen(Eigen::Vector2i(WIN_WIDTH, WIN_HEIGHT), "PSU", false);

    nanogui::mainloop();
    nanogui::shutdown();

    return 0;
}
