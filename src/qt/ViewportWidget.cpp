#include "ViewportWidget.h"
#include "ImGuiFonts.h"

#include <imgui.h>
#include <imgui_impl_opengl3.h>

#include <QFileDialog>
#include <QFileInfo>
#include <QGuiApplication>
#include <QKeyEvent>
#include <QStandardPaths>
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

bool ViewportWidget::wantsTextInput() const {
    return ready_ && ImGui::GetIO().WantTextInput;
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

// QFileDialog: native on Windows, and it returns Unicode paths, so nothing
// goes through the ANSI code page. It runs a nested event loop; a repaint
// during it is skipped by the inFrame_ guard in paintGL.
bool ViewportWidget::chooseFile(FileDialog kind, const char* title, std::string& utf8Path) {
    const FileDialogSpec& spec = fileDialogSpec(kind);
    if (lastDir_.isEmpty()) lastDir_ = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);

    QFileDialog dlg(window(), QString::fromUtf8(title ? title : spec.title), lastDir_);
    if (spec.folder) {
        dlg.setFileMode(QFileDialog::Directory);
        dlg.setOption(QFileDialog::ShowDirsOnly, true);
    } else {
        dlg.setNameFilter(QString::fromUtf8(spec.filter));
        if (spec.save) {
            dlg.setAcceptMode(QFileDialog::AcceptSave);
            dlg.setFileMode(QFileDialog::AnyFile);
            dlg.setDefaultSuffix(QString::fromUtf8(spec.defaultSuffix));
        } else {
            dlg.setAcceptMode(QFileDialog::AcceptOpen);
            dlg.setFileMode(QFileDialog::ExistingFile);
        }
    }

    utf8Path.clear();
    input_.releaseAll();
    if (dlg.exec() == QDialog::Accepted && !dlg.selectedFiles().isEmpty()) {
        const QString chosen = dlg.selectedFiles().first();
        lastDir_ = spec.folder ? chosen : QFileInfo(chosen).absolutePath();
        utf8Path = QDir::toNativeSeparators(chosen).toUtf8().toStdString();
    }
    return true;
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

    InputFrame in;
    in.viewX = 0.0f;
    in.viewY = 0.0f;   // the toolbar is a Qt widget: the whole viewport takes input
    in.viewW = (float)w;
    in.viewH = (float)h;
    input_.frame(dt, (float)w, (float)h, in);

    profiler.begin("UI+Input");
    app_.frame(dt, w, h, &in);
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

float ViewportWidget::scale() const { return (float)devicePixelRatioF(); }

void ViewportWidget::mousePressEvent(QMouseEvent* e) { input_.mouseButton(e, scale(), true, false); }
void ViewportWidget::mouseReleaseEvent(QMouseEvent* e) { input_.mouseButton(e, scale(), false, false); }
// Qt replaces the second press of a double click with this event.
void ViewportWidget::mouseDoubleClickEvent(QMouseEvent* e) { input_.mouseButton(e, scale(), true, true); }
void ViewportWidget::mouseMoveEvent(QMouseEvent* e) { input_.mouseMove(e, scale()); }
void ViewportWidget::wheelEvent(QWheelEvent* e) { input_.wheel(e); }
void ViewportWidget::keyPressEvent(QKeyEvent* e) { input_.key(e, true); }
void ViewportWidget::keyReleaseEvent(QKeyEvent* e) { input_.key(e, false); }
void ViewportWidget::focusInEvent(QFocusEvent*) {}
// Key releases made while a dock or field has the keyboard never come here.
void ViewportWidget::focusOutEvent(QFocusEvent*) { input_.releaseAll(false); }
void ViewportWidget::leaveEvent(QEvent*) { input_.leave(QGuiApplication::mouseButtons() != Qt::NoButton); }

void ViewportWidget::changeEvent(QEvent* e) {
    // Another window (a native dialog, another app) became active.
    if (e->type() == QEvent::ActivationChange && !isActiveWindow() && ready_) input_.releaseAll();
    QOpenGLWidget::changeEvent(e);
}

} // namespace shitcad
