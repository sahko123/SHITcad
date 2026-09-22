#pragma once
// App.h first: it brings in glad, which must precede any GL header Qt pulls in.
#include "App.h"
#include "ImGuiQt.h"

#include <QElapsedTimer>
#include <QOpenGLWidget>

// Qt's headers include windows.h, which defines these as empty macros.
#undef near
#undef far

namespace shitcad {

// Hosts App inside a QOpenGLWidget. Until the panels move to Qt widgets
// (Phase 5 of docs/qt-migration-plan.md), the whole existing ImGui UI is
// drawn inside this widget, fed by ImGuiQt.
class ViewportWidget : public QOpenGLWidget, public AppHost {
    Q_OBJECT
public:
    explicit ViewportWidget(QWidget* parent = nullptr);
    ~ViewportWidget() override;

    App& app() { return app_; }

    // AppHost
    void setWindowTitle(const std::string& utf8Title) override;
    void requestRedraw() override { update(); }

signals:
    // After App::frame(): widgets outside the viewport refresh from App here.
    void frameBuilt();

protected:
    void initializeGL() override;
    void paintGL() override;

    void mousePressEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;
    void mouseDoubleClickEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void wheelEvent(QWheelEvent* e) override;
    void keyPressEvent(QKeyEvent* e) override;
    void keyReleaseEvent(QKeyEvent* e) override;
    void focusInEvent(QFocusEvent* e) override;
    void focusOutEvent(QFocusEvent* e) override;
    void leaveEvent(QEvent* e) override;
    bool focusNextPrevChild(bool) override { return false; } // Tab belongs to ImGui

private:
    void teardown();

    App app_;
    ImGuiQt imgui_;
    QElapsedTimer clock_;
    bool ready_ = false;
    // A modal dialog opened during a frame (the Win32 file dialogs) runs its
    // own message loop, which can deliver another repaint while the first
    // frame is still being built. That nested paint is skipped.
    bool inFrame_ = false;
};

} // namespace shitcad
