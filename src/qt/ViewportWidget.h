#pragma once
// App.h first: it brings in glad, which must precede any GL header Qt pulls in.
#include "App.h"
#include "QtInput.h"

#include <QElapsedTimer>
#include <QOpenGLWidget>

// Qt's headers include windows.h, which defines these as empty macros.
#undef near
#undef far

namespace shitcad {

// Hosts App inside a QOpenGLWidget: feeds it the viewport's input
// (InputCollector), runs a frame per repaint, and paints App's overlay with
// QPainter over the 3D scene. The panels are separate Qt widgets.
class ViewportWidget : public QOpenGLWidget, public AppHost {
    Q_OBJECT
public:
    explicit ViewportWidget(QWidget* parent = nullptr);
    ~ViewportWidget() override;

    App& app() { return app_; }

    // AppHost
    void setWindowTitle(const std::string& utf8Title) override;
    void requestRedraw() override { update(); }
    bool chooseFile(FileDialog kind, const char* title, std::string& utf8Path) override;

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
    void changeEvent(QEvent* e) override;
    bool focusNextPrevChild(bool) override { return false; } // Tab does not leave the view

private:
    void teardown();
    float scale() const;

    App app_;
    InputCollector input_;
    QElapsedTimer clock_;
    bool ready_ = false;
    // A modal dialog opened during a frame (the Win32 file dialogs) runs its
    // own message loop, which can deliver another repaint while the first
    // frame is still being built. That nested paint is skipped.
    bool inFrame_ = false;
    QString lastDir_;  // where the last file dialog ended up
};

} // namespace shitcad
