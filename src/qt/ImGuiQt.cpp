#include "ImGuiQt.h"

#include <imgui.h>

#include <QClipboard>
#include <QCursor>
#include <QGuiApplication>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QWidget>

#include <cfloat>
#include <cmath>
#include <string>

namespace shitcad {

namespace {

// Qt::Key -> ImGuiKey. Keys Qt reports as the shifted symbol are mapped back
// to the key that produces them on a US layout, as ImGui expects the key, not
// the character (the character arrives separately as text).
ImGuiKey toImGuiKey(int k, bool keypad) {
    if (k >= Qt::Key_A && k <= Qt::Key_Z) return (ImGuiKey)(ImGuiKey_A + (k - Qt::Key_A));
    if (k >= Qt::Key_0 && k <= Qt::Key_9)
        return keypad ? (ImGuiKey)(ImGuiKey_Keypad0 + (k - Qt::Key_0)) : (ImGuiKey)(ImGuiKey_0 + (k - Qt::Key_0));
    if (k >= Qt::Key_F1 && k <= Qt::Key_F24) return (ImGuiKey)(ImGuiKey_F1 + (k - Qt::Key_F1));
    switch (k) {
        case Qt::Key_Tab: case Qt::Key_Backtab: return ImGuiKey_Tab;
        case Qt::Key_Left: return ImGuiKey_LeftArrow;
        case Qt::Key_Right: return ImGuiKey_RightArrow;
        case Qt::Key_Up: return ImGuiKey_UpArrow;
        case Qt::Key_Down: return ImGuiKey_DownArrow;
        case Qt::Key_PageUp: return ImGuiKey_PageUp;
        case Qt::Key_PageDown: return ImGuiKey_PageDown;
        case Qt::Key_Home: return ImGuiKey_Home;
        case Qt::Key_End: return ImGuiKey_End;
        case Qt::Key_Insert: return ImGuiKey_Insert;
        case Qt::Key_Delete: return ImGuiKey_Delete;
        case Qt::Key_Backspace: return ImGuiKey_Backspace;
        case Qt::Key_Space: return ImGuiKey_Space;
        case Qt::Key_Return: return ImGuiKey_Enter;
        case Qt::Key_Enter: return keypad ? ImGuiKey_KeypadEnter : ImGuiKey_Enter;
        case Qt::Key_Escape: return ImGuiKey_Escape;
        case Qt::Key_Apostrophe: case Qt::Key_QuoteDbl: return ImGuiKey_Apostrophe;
        case Qt::Key_Comma: case Qt::Key_Less: return ImGuiKey_Comma;
        case Qt::Key_Minus: return keypad ? ImGuiKey_KeypadSubtract : ImGuiKey_Minus;
        case Qt::Key_Underscore: return ImGuiKey_Minus;
        case Qt::Key_Period: return keypad ? ImGuiKey_KeypadDecimal : ImGuiKey_Period;
        case Qt::Key_Greater: return ImGuiKey_Period;
        case Qt::Key_Slash: return keypad ? ImGuiKey_KeypadDivide : ImGuiKey_Slash;
        case Qt::Key_Question: return ImGuiKey_Slash;
        case Qt::Key_Semicolon: case Qt::Key_Colon: return ImGuiKey_Semicolon;
        case Qt::Key_Equal: return keypad ? ImGuiKey_KeypadEqual : ImGuiKey_Equal;
        case Qt::Key_Plus: return keypad ? ImGuiKey_KeypadAdd : ImGuiKey_Equal;
        case Qt::Key_Asterisk: return keypad ? ImGuiKey_KeypadMultiply : ImGuiKey_8;
        case Qt::Key_BracketLeft: case Qt::Key_BraceLeft: return ImGuiKey_LeftBracket;
        case Qt::Key_BracketRight: case Qt::Key_BraceRight: return ImGuiKey_RightBracket;
        case Qt::Key_Backslash: case Qt::Key_Bar: return ImGuiKey_Backslash;
        case Qt::Key_QuoteLeft: case Qt::Key_AsciiTilde: return ImGuiKey_GraveAccent;
        case Qt::Key_Exclam: return ImGuiKey_1;
        case Qt::Key_At: return ImGuiKey_2;
        case Qt::Key_NumberSign: return ImGuiKey_3;
        case Qt::Key_Dollar: return ImGuiKey_4;
        case Qt::Key_Percent: return ImGuiKey_5;
        case Qt::Key_AsciiCircum: return ImGuiKey_6;
        case Qt::Key_Ampersand: return ImGuiKey_7;
        case Qt::Key_ParenLeft: return ImGuiKey_9;
        case Qt::Key_ParenRight: return ImGuiKey_0;
        case Qt::Key_CapsLock: return ImGuiKey_CapsLock;
        case Qt::Key_ScrollLock: return ImGuiKey_ScrollLock;
        case Qt::Key_NumLock: return ImGuiKey_NumLock;
        case Qt::Key_Print: return ImGuiKey_PrintScreen;
        case Qt::Key_Pause: return ImGuiKey_Pause;
        case Qt::Key_Menu: return ImGuiKey_Menu;
        case Qt::Key_Shift: return ImGuiKey_LeftShift;
        case Qt::Key_Control: return ImGuiKey_LeftCtrl;
        case Qt::Key_Alt: return ImGuiKey_LeftAlt;
        case Qt::Key_Meta: return ImGuiKey_LeftSuper;
        default: return ImGuiKey_None;
    }
}

int toImGuiButton(Qt::MouseButton b) {
    switch (b) {
        case Qt::LeftButton: return 0;
        case Qt::RightButton: return 1;
        case Qt::MiddleButton: return 2;
        case Qt::BackButton: return 3;
        case Qt::ForwardButton: return 4;
        default: return -1;
    }
}

std::string g_clipboard; // keeps the returned text alive, as ImGui requires

const char* getClipboard(ImGuiContext*) {
    g_clipboard = QGuiApplication::clipboard()->text().toStdString(); // UTF-8
    return g_clipboard.c_str();
}

void setClipboard(ImGuiContext*, const char* text) {
    QGuiApplication::clipboard()->setText(QString::fromUtf8(text));
}

} // namespace

void ImGuiQt::init(QWidget* widget) {
    widget_ = widget;
    ImGuiIO& io = ImGui::GetIO();
    io.BackendPlatformName = "imgui_impl_qt (SHITcad)";
    io.BackendFlags |= ImGuiBackendFlags_HasMouseCursors;
    ImGuiPlatformIO& pio = ImGui::GetPlatformIO();
    pio.Platform_GetClipboardTextFn = getClipboard;
    pio.Platform_SetClipboardTextFn = setClipboard;
}

void ImGuiQt::shutdown() {
    ImGuiIO& io = ImGui::GetIO();
    io.BackendPlatformName = nullptr;
    io.BackendFlags &= ~ImGuiBackendFlags_HasMouseCursors;
    widget_ = nullptr;
}

float ImGuiQt::scale() const {
    return widget_ ? (float)widget_->devicePixelRatioF() : 1.0f;
}

void ImGuiQt::newFrame(float dt) {
    ImGuiIO& io = ImGui::GetIO();
    const float s = scale();
    io.DisplaySize = ImVec2(std::round(widget_->width() * s), std::round(widget_->height() * s));
    io.DisplayFramebufferScale = ImVec2(1.0f, 1.0f); // DisplaySize is already in pixels
    io.DeltaTime = dt > 0.0f ? dt : 1.0f / 60.0f;

    // Cursor
    if (io.ConfigFlags & ImGuiConfigFlags_NoMouseCursorChange) return;
    const int c = io.MouseDrawCursor ? (int)ImGuiMouseCursor_None : (int)ImGui::GetMouseCursor();
    if (c == lastCursor_) return;
    lastCursor_ = c;
    Qt::CursorShape shape = Qt::ArrowCursor;
    switch (c) {
        case ImGuiMouseCursor_None: shape = Qt::BlankCursor; break;
        case ImGuiMouseCursor_TextInput: shape = Qt::IBeamCursor; break;
        case ImGuiMouseCursor_ResizeAll: shape = Qt::SizeAllCursor; break;
        case ImGuiMouseCursor_ResizeNS: shape = Qt::SizeVerCursor; break;
        case ImGuiMouseCursor_ResizeEW: shape = Qt::SizeHorCursor; break;
        case ImGuiMouseCursor_ResizeNESW: shape = Qt::SizeBDiagCursor; break;
        case ImGuiMouseCursor_ResizeNWSE: shape = Qt::SizeFDiagCursor; break;
        case ImGuiMouseCursor_Hand: shape = Qt::PointingHandCursor; break;
        case ImGuiMouseCursor_NotAllowed: shape = Qt::ForbiddenCursor; break;
        default: break;
    }
    widget_->setCursor(shape);
}

void ImGuiQt::updateModifiers(int m) {
    ImGuiIO& io = ImGui::GetIO();
    io.AddKeyEvent(ImGuiMod_Ctrl, (m & Qt::ControlModifier) != 0);
    io.AddKeyEvent(ImGuiMod_Shift, (m & Qt::ShiftModifier) != 0);
    io.AddKeyEvent(ImGuiMod_Alt, (m & Qt::AltModifier) != 0);
    io.AddKeyEvent(ImGuiMod_Super, (m & Qt::MetaModifier) != 0);
}

void ImGuiQt::mouseMove(QMouseEvent* e) {
    const float s = scale();
    ImGui::GetIO().AddMousePosEvent((float)e->position().x() * s, (float)e->position().y() * s);
}

void ImGuiQt::mouseButton(QMouseEvent* e, bool down) {
    updateModifiers((int)e->modifiers());
    mouseMove(e);
    const int b = toImGuiButton(e->button());
    if (b >= 0) ImGui::GetIO().AddMouseButtonEvent(b, down);
}

void ImGuiQt::wheel(QWheelEvent* e) {
    updateModifiers((int)e->modifiers());
    // One notch is 120 eighths of a degree; GLFW reports it as 1.0.
    const QPoint d = e->angleDelta();
    ImGui::GetIO().AddMouseWheelEvent((float)d.x() / 120.0f, (float)d.y() / 120.0f);
}

void ImGuiQt::key(QKeyEvent* e, bool down) {
    // ImGui derives key repeat from how long a key is held, so OS auto-repeat
    // is dropped (the GLFW backend ignores GLFW_REPEAT the same way). Text
    // from repeats is still typed.
    ImGuiIO& io = ImGui::GetIO();
    updateModifiers((int)e->modifiers());
    if (!e->isAutoRepeat()) {
        const unsigned scan = e->nativeScanCode();
        ImGuiKey k = ImGuiKey_None;
        if (down) {
            k = toImGuiKey(e->key(), (e->modifiers() & Qt::KeypadModifier) != 0);
            if (scan) pressedKeys_[scan] = (int)k;
        } else {
            auto it = scan ? pressedKeys_.find(scan) : pressedKeys_.end();
            if (it != pressedKeys_.end()) { k = (ImGuiKey)it->second; pressedKeys_.erase(it); }
            else k = toImGuiKey(e->key(), (e->modifiers() & Qt::KeypadModifier) != 0);
        }
        if (k != ImGuiKey_None) io.AddKeyEvent(k, down);
    }
    if (down) {
        for (QChar ch : e->text()) {
            const ushort u = ch.unicode();
            if (u >= 32 && u != 127) io.AddInputCharacterUTF16(u); // no control characters (Ctrl+Z)
        }
    }
}

void ImGuiQt::focus(bool in) {
    ImGui::GetIO().AddFocusEvent(in);
    if (!in) releaseAll(); // releases made while another window has focus never arrive here
}

void ImGuiQt::releaseAll() {
    ImGuiIO& io = ImGui::GetIO();
    for (const auto& kv : pressedKeys_) io.AddKeyEvent((ImGuiKey)kv.second, false);
    pressedKeys_.clear();
    for (ImGuiKey m : {ImGuiMod_Ctrl, ImGuiMod_Shift, ImGuiMod_Alt, ImGuiMod_Super}) io.AddKeyEvent(m, false);
    for (int b = 0; b < ImGuiMouseButton_COUNT; b++) io.AddMouseButtonEvent(b, false);
}

void ImGuiQt::leave() {
    if (QGuiApplication::mouseButtons() == Qt::NoButton)
        ImGui::GetIO().AddMousePosEvent(-FLT_MAX, -FLT_MAX);
}

} // namespace shitcad
