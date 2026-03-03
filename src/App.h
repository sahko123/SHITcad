#pragma once
#include "Canvas.h"
#include "SketchData.h"
#include "Tools.h"

struct GLFWwindow;

namespace shitcad {

class App {
public:
    bool init();
    void run();
    void shutdown();

private:
    GLFWwindow* window_ = nullptr;
    Canvas canvas_;
    Sketch sketch_;

    void renderFrame();
};

} // namespace shitcad
