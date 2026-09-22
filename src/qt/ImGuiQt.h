#pragma once
#include <unordered_map>

class QWidget;
class QMouseEvent;
class QWheelEvent;
class QKeyEvent;

namespace shitcad {

// Dear ImGui platform backend for a Qt widget: feeds Qt input to ImGuiIO and
// sets the cursor ImGui asks for. Rendering stays with imgui_impl_opengl3.
//
// Everything is in *device* pixels (logical * devicePixelRatio), which is the
// space the GLFW backend used on Windows and the one App's projection,
// picking and overlays work in.
class ImGuiQt {
public:
    void init(QWidget* widget);
    void shutdown();
    void newFrame(float dt);

    void mouseMove(QMouseEvent* e);
    void mouseButton(QMouseEvent* e, bool down);
    void wheel(QWheelEvent* e);
    void key(QKeyEvent* e, bool down);
    void focus(bool in);
    void leave();
    // Release every key, modifier and mouse button ImGui thinks is held, as
    // GLFW does when its window loses focus. Needed before a modal dialog:
    // the releases go to the dialog, and a key ImGui still thinks is down
    // auto-repeats once the dialog closes (Ctrl+O reopened Open).
    void releaseAll();

private:
    float scale() const;
    void updateModifiers(int qtModifiers);

    QWidget* widget_ = nullptr;
    int lastCursor_ = -2;
    // Scan code -> ImGuiKey from the press, so the release matches it even if
    // a modifier changed in between (Shift+1 presses '!', releases '1').
    std::unordered_map<unsigned, int> pressedKeys_;
};

} // namespace shitcad
