#include "ViewportWidget.h"
#include "QtOverlay.h"


#include <QDragEnterEvent>
#include <QDropEvent>
#include <QFileDialog>
#include <QMimeData>
#include <QUrl>
#include <QFileInfo>
#include <QGuiApplication>
#include <QKeyEvent>
#include <QStandardPaths>
#include <QMouseEvent>
#include <QOpenGLContext>
#include <QPainter>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace shitcad {

namespace {

// The GL state QPainter's GL engine may change, saved before the overlay is
// painted and put back after it, so App's 3D pass starts each frame from the
// state it left.
struct GlStateGuard {
    GLint program, vao, arrayBuffer, activeTexture, texture2D, unpackAlign;
    GLint viewport[4], scissorBox[4];
    GLint blendSrcRgb, blendDstRgb, blendSrcAlpha, blendDstAlpha, blendEqRgb, blendEqAlpha;
    GLint depthFunc, cullMode, frontFace, stencilFunc, stencilRef, stencilValueMask, stencilWriteMask;
    GLint stencilFail, stencilPassDepthFail, stencilPassDepthPass;
    GLboolean depthMask, colorMask[4];
    GLboolean blend, depthTest, cullFace, scissorTest, stencilTest, polygonOffsetFill;
    GLfloat polygonOffsetFactor, polygonOffsetUnits;

    GlStateGuard() {
        glGetIntegerv(GL_CURRENT_PROGRAM, &program);
        glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &vao);
        glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &arrayBuffer);
        glGetIntegerv(GL_ACTIVE_TEXTURE, &activeTexture);
        glActiveTexture(GL_TEXTURE0);
        glGetIntegerv(GL_TEXTURE_BINDING_2D, &texture2D);
        glGetIntegerv(GL_UNPACK_ALIGNMENT, &unpackAlign);
        glGetIntegerv(GL_VIEWPORT, viewport);
        glGetIntegerv(GL_SCISSOR_BOX, scissorBox);
        glGetIntegerv(GL_BLEND_SRC_RGB, &blendSrcRgb);
        glGetIntegerv(GL_BLEND_DST_RGB, &blendDstRgb);
        glGetIntegerv(GL_BLEND_SRC_ALPHA, &blendSrcAlpha);
        glGetIntegerv(GL_BLEND_DST_ALPHA, &blendDstAlpha);
        glGetIntegerv(GL_BLEND_EQUATION_RGB, &blendEqRgb);
        glGetIntegerv(GL_BLEND_EQUATION_ALPHA, &blendEqAlpha);
        glGetIntegerv(GL_DEPTH_FUNC, &depthFunc);
        glGetIntegerv(GL_CULL_FACE_MODE, &cullMode);
        glGetIntegerv(GL_FRONT_FACE, &frontFace);
        glGetIntegerv(GL_STENCIL_FUNC, &stencilFunc);
        glGetIntegerv(GL_STENCIL_REF, &stencilRef);
        glGetIntegerv(GL_STENCIL_VALUE_MASK, &stencilValueMask);
        glGetIntegerv(GL_STENCIL_WRITEMASK, &stencilWriteMask);
        glGetIntegerv(GL_STENCIL_FAIL, &stencilFail);
        glGetIntegerv(GL_STENCIL_PASS_DEPTH_FAIL, &stencilPassDepthFail);
        glGetIntegerv(GL_STENCIL_PASS_DEPTH_PASS, &stencilPassDepthPass);
        glGetBooleanv(GL_DEPTH_WRITEMASK, &depthMask);
        glGetBooleanv(GL_COLOR_WRITEMASK, colorMask);
        blend = glIsEnabled(GL_BLEND);
        depthTest = glIsEnabled(GL_DEPTH_TEST);
        cullFace = glIsEnabled(GL_CULL_FACE);
        scissorTest = glIsEnabled(GL_SCISSOR_TEST);
        stencilTest = glIsEnabled(GL_STENCIL_TEST);
        polygonOffsetFill = glIsEnabled(GL_POLYGON_OFFSET_FILL);
        glGetFloatv(GL_POLYGON_OFFSET_FACTOR, &polygonOffsetFactor);
        glGetFloatv(GL_POLYGON_OFFSET_UNITS, &polygonOffsetUnits);
    }

    ~GlStateGuard() {
        auto set = [](GLenum cap, GLboolean on) { if (on) glEnable(cap); else glDisable(cap); };
        glUseProgram(program);
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, arrayBuffer);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texture2D);
        glActiveTexture(activeTexture);
        glPixelStorei(GL_UNPACK_ALIGNMENT, unpackAlign);
        glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
        glScissor(scissorBox[0], scissorBox[1], scissorBox[2], scissorBox[3]);
        glBlendFuncSeparate(blendSrcRgb, blendDstRgb, blendSrcAlpha, blendDstAlpha);
        glBlendEquationSeparate(blendEqRgb, blendEqAlpha);
        glDepthFunc(depthFunc);
        glCullFace(cullMode);
        glFrontFace(frontFace);
        glStencilFunc(stencilFunc, stencilRef, stencilValueMask);
        glStencilMask(stencilWriteMask);
        glStencilOp(stencilFail, stencilPassDepthFail, stencilPassDepthPass);
        glDepthMask(depthMask);
        glColorMask(colorMask[0], colorMask[1], colorMask[2], colorMask[3]);
        set(GL_BLEND, blend);
        set(GL_DEPTH_TEST, depthTest);
        set(GL_CULL_FACE, cullFace);
        set(GL_SCISSOR_TEST, scissorTest);
        set(GL_STENCIL_TEST, stencilTest);
        set(GL_POLYGON_OFFSET_FILL, polygonOffsetFill);
        glPolygonOffset(polygonOffsetFactor, polygonOffsetUnits);
    }
};

} // namespace

ViewportWidget::ViewportWidget(QWidget* parent) : QOpenGLWidget(parent) {
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);  // hover (snaps, highlights) needs moves without a button held
    setMinimumSize(320, 240);
    // Render continuously, as the GLFW host does: each finished frame asks
    // for the next. On-demand rendering can come later (see App::post).
    connect(this, &QOpenGLWidget::frameSwapped, this, qOverload<>(&QWidget::update));
    setAcceptDrops(true);
}

namespace {

// Local files among a drag's URLs that App can import, as UTF-8.
std::vector<std::string> importablePaths(const QMimeData* mime) {
    std::vector<std::string> out;
    if (!mime || !mime->hasUrls()) return out;
    for (const QUrl& url : mime->urls()) {
        if (!url.isLocalFile()) continue;
        const QString suffix = QFileInfo(url.toLocalFile()).suffix().toLower();
        if (suffix == "step" || suffix == "stp" || suffix == "igs" || suffix == "iges" || suffix == "stl")
            out.push_back(url.toLocalFile().toUtf8().toStdString());
    }
    return out;
}

} // namespace

void ViewportWidget::dragEnterEvent(QDragEnterEvent* e) {
    // Refused (the no-drop cursor) while sketching or running a tool: the
    // import would replay the model under it. App::importFile checks again.
    if (app_.canImport() && !importablePaths(e->mimeData()).empty()) e->acceptProposedAction();
}

void ViewportWidget::dropEvent(QDropEvent* e) {
    const std::vector<std::string> paths = importablePaths(e->mimeData());
    if (paths.empty()) return;
    e->acceptProposedAction();
    // One import dialog at a time: the first file. The rest would replace it.
    const std::string path = paths.front();
    App* a = &app_;
    app_.post([a, path] { a->importFile(path); });
}

ViewportWidget::~ViewportWidget() {
    teardown();
}

void ViewportWidget::teardown() {
    if (!ready_) return;
    ready_ = false;
    makeCurrent();
    app_.shutdown();
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

    const float dpi = std::max(1.0f, (float)devicePixelRatioF());
    // The overlay is drawn with QPainter, and its text measured with the same font.
    setOverlayScale(dpi);
    app_.setOverlayMeasure(overlayMeasure);
    if (!app_.init(this, dpi)) {
        fprintf(stderr, "Failed to initialize SHITcad\n");
        return;
    }

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

    const float s = (float)devicePixelRatioF();
    const int w = (int)std::lround(width() * s), h = (int)std::lround(height() * s);

    InputFrame in;
    in.viewX = 0.0f;
    in.viewY = 0.0f;   // the toolbar is a Qt widget: the whole viewport takes input
    in.viewW = (float)w;
    in.viewH = (float)h;
    input_.frame(dt, (float)w, (float)h, in);

    profiler.begin("UI+Input");
    app_.frame(dt, w, h, in);
    profiler.end();
    emit frameBuilt();

    profiler.begin("Render3D");
    app_.paint();
    profiler.end();

    profiler.begin("Overlay");
    {
        const GlStateGuard keep;
        // QPainter's GL engine expects GL's defaults, not what the 3D pass
        // left (depth and stencil tests, masks): without this its fills and
        // strokes vanish and only text draws.
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_STENCIL_TEST);
        glDisable(GL_SCISSOR_TEST);
        glDisable(GL_CULL_FACE);
        glDisable(GL_POLYGON_OFFSET_FILL);
        glDepthMask(GL_TRUE);
        glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
        glStencilMask(0xFF);
        glUseProgram(0);
        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        QPainter painter(this);
        drawOverlay(painter, app_.overlay(), (float)devicePixelRatioF());
    }
    profiler.end();

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
