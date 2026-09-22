#include "ViewportWidget.h"
#include "ImGuiFonts.h"

#include <imgui.h>
#include <imgui_impl_opengl3.h>

#include <QKeyEvent>
#include <QMouseEvent>
#include <QOpenGLContext>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace shitcad {

ViewportWidget::ViewportWidget(QWidget* parent) : QOpenGLWidget(parent) {
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);  // hover (snaps, highlights) needs moves without a button held
    setMinimumSize(320, 240);
    // Render continuously, as the GLFW host does: each finished frame asks
    // for the next. On-demand rendering can come later (see App::post).
    connect(this, &QOpenGLWidget::frameSwapped, this, qOverload<>(&QWidget::update));
}

ViewportWidget::~ViewportWidget() {
    teardown();
}

void ViewportWidget::teardown() {
    if (!ready_) return;
    ready_ = false;
    makeCurrent();
    app_.shutdown();
    ImGui_ImplOpenGL3_Shutdown();
    imgui_.shutdown();
    ImGui::DestroyContext();
    doneCurrent();
}

void ViewportWidget::setWindowTitle(const std::string& utf8Title) {
    window()->setWindowTitle(QString::fromStdString(utf8Title));
}

void ViewportWidget::initializeGL() {
    auto load = [](const char* name) -> GLADapiproc {
        return (GLADapiproc)QOpenGLContext::currentContext()->getProcAddress(name);
    };
    if (gladLoadGL(load) == 0) {
        fprintf(stderr, "Failed to initialize OpenGL loader\n");
        return;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    const float dpi = std::max(1.0f, (float)devicePixelRatioF());
    loadImGuiFonts(dpi);

    if (!app_.init(this, dpi)) {
        fprintf(stderr, "Failed to initialize SHITcad\n");
        return;
    }
    imgui_.init(this);
    ImGui_ImplOpenGL3_Init("#version 330");

    // The context dies with the widget's window; tear down while it exists.
    connect(context(), &QOpenGLContext::aboutToBeDestroyed, this, [this] { teardown(); });
    clock_.start();
    ready_ = true;
}

void ViewportWidget::paintGL() {
    if (!ready_ || inFrame_) return;
    inFrame_ = true;

    auto& profiler = app_.profiler();
    profiler.beginFrame();

    const float dt = (float)clock_.nsecsElapsed() * 1e-9f;
    clock_.restart();

    imgui_.newFrame(dt);
    ImGui_ImplOpenGL3_NewFrame();
    ImGui::NewFrame();

    const float s = (float)devicePixelRatioF();
    const int w = (int)std::lround(width() * s), h = (int)std::lround(height() * s);

    profiler.begin("UI+Input");
    app_.frame(dt, w, h);
    profiler.end();
    emit frameBuilt();

    profiler.begin("Render3D");
    app_.paint();
    profiler.end();

    ImGui::Render();
    profiler.begin("ImGuiDraw");
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    profiler.end();

    profiler.recordFrameEnd();
    profiler.endFrame();
    inFrame_ = false;
}

void ViewportWidget::mousePressEvent(QMouseEvent* e) { imgui_.mouseButton(e, true); }
void ViewportWidget::mouseReleaseEvent(QMouseEvent* e) { imgui_.mouseButton(e, false); }
// Qt replaces the second press of a double click with this event; ImGui
// times double clicks itself and needs it as an ordinary press.
void ViewportWidget::mouseDoubleClickEvent(QMouseEvent* e) { imgui_.mouseButton(e, true); }
void ViewportWidget::mouseMoveEvent(QMouseEvent* e) { imgui_.mouseMove(e); }
void ViewportWidget::wheelEvent(QWheelEvent* e) { imgui_.wheel(e); }
void ViewportWidget::keyPressEvent(QKeyEvent* e) { imgui_.key(e, true); }
void ViewportWidget::keyReleaseEvent(QKeyEvent* e) { imgui_.key(e, false); }
void ViewportWidget::focusInEvent(QFocusEvent*) { imgui_.focus(true); }
void ViewportWidget::focusOutEvent(QFocusEvent*) { imgui_.focus(false); }
void ViewportWidget::leaveEvent(QEvent*) { imgui_.leave(); }

} // namespace shitcad
