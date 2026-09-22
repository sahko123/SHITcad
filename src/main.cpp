// GLFW + Dear ImGui host: owns the window, the GL context, the ImGui context
// and the event loop, and drives App through AppHost (see AppHost.h).
#include "CrashLogger.h"
#include "App.h"
#include "ImGuiFonts.h"

#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <cstdio>

namespace {

class GlfwHost : public shitcad::AppHost {
public:
    bool init() {
        if (!glfwInit()) {
            fprintf(stderr, "Failed to initialize GLFW\n");
            return false;
        }

        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_MAXIMIZED, GLFW_TRUE);

        window_ = glfwCreateWindow(1280, 720, "SHITcad", nullptr, nullptr);
        if (!window_) {
            fprintf(stderr, "Failed to create GLFW window\n");
            glfwTerminate();
            return false;
        }

        glfwMakeContextCurrent(window_);
        glfwSwapInterval(1);

        if (gladLoadGL(glfwGetProcAddress) == 0) {
            fprintf(stderr, "Failed to initialize OpenGL loader\n");
            return false;
        }

        IMGUI_CHECKVERSION();
        ImGui::CreateContext();

        // DPI scaling: query monitor content scale
        float xscale = 1.0f, yscale = 1.0f;
        glfwGetWindowContentScale(window_, &xscale, &yscale);
        dpiScale_ = xscale > yscale ? xscale : yscale;
        if (dpiScale_ < 1.0f) dpiScale_ = 1.0f;

        shitcad::loadImGuiFonts(dpiScale_);
        return true;
    }

    // App::init (style, GL resources) runs between these two.
    void initBackends() {
        ImGui_ImplGlfw_InitForOpenGL(window_, true);
        ImGui_ImplOpenGL3_Init("#version 330");
    }

    void run(shitcad::App& app) {
        auto& profiler = app.profiler();
        double lastTime = glfwGetTime();

        while (!glfwWindowShouldClose(window_)) {
            profiler.beginFrame();

            profiler.begin("PollEvents");
            glfwPollEvents();
            profiler.end();

            double now = glfwGetTime();
            float dt = (float)(now - lastTime);
            lastTime = now;

            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();

            int w, h;
            glfwGetFramebufferSize(window_, &w, &h);

            profiler.begin("UI+Input");
            app.frame(dt, w, h);
            profiler.end();

            profiler.begin("Render3D");
            app.paint();
            profiler.end();

            ImGui::Render();

            profiler.begin("ImGuiDraw");
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
            profiler.end();

            profiler.begin("SwapBuffers");
            glfwSwapBuffers(window_);
            profiler.end();

            profiler.recordFrameEnd();
            profiler.endFrame();
        }
    }

    void shutdown() {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();

        if (window_) {
            glfwDestroyWindow(window_);
            window_ = nullptr;
        }
        glfwTerminate();
    }

    float dpiScale() const { return dpiScale_; }

    void setWindowTitle(const std::string& utf8Title) override {
        glfwSetWindowTitle(window_, utf8Title.c_str()); // GLFW takes UTF-8
    }

private:
    GLFWwindow* window_ = nullptr;
    float dpiScale_ = 1.0f;
};

} // namespace

int main() {
    shitcad::initCrashLogger();

    GlfwHost host;
    shitcad::App app;
    if (!host.init() || !app.init(&host, host.dpiScale())) {
        fprintf(stderr, "Failed to initialize SHITcad\n");
        return 1;
    }
    host.initBackends();
    host.run(app);
    app.shutdown();
    host.shutdown();
    return 0;
}
