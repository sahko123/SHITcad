#include "QtInput.h"

#include <QKeyEvent>
#include <QMouseEvent>
#include <QWheelEvent>

#include <algorithm>
#include <cfloat>

namespace shitcad {

namespace {

constexpr float kRepeatDelay = 0.275f;   // ImGui's io.KeyRepeatDelay
constexpr float kRepeatRate = 0.050f;    // ImGui's io.KeyRepeatRate

// Qt key -> the viewport's Key, or -1 for keys the handlers do not use.
int toKey(int k, bool keypad) {
    if (k >= Qt::Key_0 && k <= Qt::Key_9)
        return keypad ? (int)Key::Keypad0 + (k - Qt::Key_0) : (int)Key::Num0 + (k - Qt::Key_0);
    switch (k) {
        case Qt::Key_A: return (int)Key::A;
        case Qt::Key_C: return (int)Key::C;
        case Qt::Key_D: return (int)Key::D;
        case Qt::Key_E: return (int)Key::E;
        case Qt::Key_F: return (int)Key::F;
        case Qt::Key_L: return (int)Key::L;
        case Qt::Key_N: return (int)Key::N;
        case Qt::Key_O: return (int)Key::O;
        case Qt::Key_P: return (int)Key::P;
        case Qt::Key_R: return (int)Key::R;
        case Qt::Key_S: return (int)Key::S;
        case Qt::Key_T: return (int)Key::T;
        case Qt::Key_V: return (int)Key::V;
        case Qt::Key_Y: return (int)Key::Y;
        case Qt::Key_Z: return (int)Key::Z;
        case Qt::Key_Period: return keypad ? (int)Key::KeypadDecimal : (int)Key::Period;
        case Qt::Key_Greater: return (int)Key::Period;
        case Qt::Key_Return: return (int)Key::Enter;
        case Qt::Key_Enter: return keypad ? (int)Key::KeypadEnter : (int)Key::Enter;
        case Qt::Key_Escape: return (int)Key::Escape;
        case Qt::Key_Delete: return (int)Key::Delete;
        case Qt::Key_Backspace: return (int)Key::Backspace;
        // Shifted digits on a US layout arrive as their symbol: the key is still the digit.
        case Qt::Key_Exclam: return (int)Key::Num1;
        case Qt::Key_At: return (int)Key::Num2;
        case Qt::Key_NumberSign: return (int)Key::Num3;
        case Qt::Key_Dollar: return (int)Key::Num4;
        case Qt::Key_Percent: return (int)Key::Num5;
        case Qt::Key_AsciiCircum: return (int)Key::Num6;
        case Qt::Key_Ampersand: return (int)Key::Num7;
        case Qt::Key_Asterisk: return keypad ? -1 : (int)Key::Num8;
        case Qt::Key_ParenLeft: return (int)Key::Num9;
        case Qt::Key_ParenRight: return (int)Key::Num0;
        default: return -1;
    }
}

int toButton(Qt::MouseButton b) {
    switch (b) {
        case Qt::LeftButton: return (int)MouseButton::Left;
        case Qt::RightButton: return (int)MouseButton::Right;
        case Qt::MiddleButton: return (int)MouseButton::Middle;
        default: return -1;
    }
}

// How many times a key held from t0 to t1 seconds repeats (ImGui's
// CalcTypematicRepeatAmount).
int repeats(float t0, float t1) {
    if (t1 == 0.0f) return 1;
    if (t0 >= t1) return 0;
    const int c0 = t0 < kRepeatDelay ? -1 : (int)((t0 - kRepeatDelay) / kRepeatRate);
    const int c1 = t1 < kRepeatDelay ? -1 : (int)((t1 - kRepeatDelay) / kRepeatRate);
    return c1 - c0;
}

bool valid(float v) { return v > -FLT_MAX * 0.5f; }

} // namespace

void InputCollector::mods(int m) {
    Event e{Ev::Mods};
    e.shift = (m & Qt::ShiftModifier) != 0;
    e.ctrl = (m & Qt::ControlModifier) != 0;
    e.alt = (m & Qt::AltModifier) != 0;
    queue_.push_back(e);
}

void InputCollector::mouseMove(QMouseEvent* e, float scale) {
    Event ev{Ev::Pos};
    ev.x = (float)e->position().x() * scale;
    ev.y = (float)e->position().y() * scale;
    queue_.push_back(ev);
}

void InputCollector::mouseButton(QMouseEvent* e, float scale, bool down, bool doubleClick) {
    mods((int)e->modifiers());
    mouseMove(e, scale);
    const int b = toButton(e->button());
    if (b < 0) return;
    Event ev{Ev::Button};
    ev.index = b;
    ev.down = down;
    ev.dbl = doubleClick;
    queue_.push_back(ev);
}

void InputCollector::wheel(QWheelEvent* e) {
    mods((int)e->modifiers());
    Event ev{Ev::Wheel};
    ev.y = (float)e->angleDelta().y() / 120.0f;   // one notch is 1.0, as GLFW reports it
    queue_.push_back(ev);
}

void InputCollector::key(QKeyEvent* e, bool down) {
    mods((int)e->modifiers());
    if (!e->isAutoRepeat()) {
        const unsigned scan = e->nativeScanCode();
        const bool keypad = (e->modifiers() & Qt::KeypadModifier) != 0;
        int k = -1;
        if (down) {
            k = toKey(e->key(), keypad);
            if (scan) pressedKeys_[scan] = k;
        } else {
            // Released as whatever it was pressed as (Shift+1 presses '!', releases '1').
            auto it = scan ? pressedKeys_.find(scan) : pressedKeys_.end();
            if (it != pressedKeys_.end()) { k = it->second; pressedKeys_.erase(it); }
            else k = toKey(e->key(), keypad);
        }
        if (k >= 0) {
            Event ev{Ev::Key};
            ev.index = k;
            ev.down = down;
            queue_.push_back(ev);
        }
    }
    if (down) {
        for (QChar ch : e->text()) {
            const char16_t u = ch.unicode();
            if (u < 32 || u == 127) continue;   // no control characters (Ctrl+Z)
            Event ev{Ev::Char};
            ev.ch = u;
            queue_.push_back(ev);
        }
    }
}

void InputCollector::leave(bool buttonHeld) {
    if (buttonHeld) return;   // a drag keeps its position outside the view
    Event ev{Ev::Pos};
    ev.x = ev.y = -FLT_MAX;
    queue_.push_back(ev);
}

void InputCollector::releaseAll(bool buttons) {
    for (const auto& kv : pressedKeys_) {
        if (kv.second < 0) continue;
        Event ev{Ev::Key};
        ev.index = kv.second;
        queue_.push_back(ev);
    }
    pressedKeys_.clear();
    queue_.push_back(Event{Ev::Mods});
    if (!buttons) return;
    for (int b = 0; b < (int)MouseButton::Count; b++) {
        Event ev{Ev::Button};
        ev.index = b;
        queue_.push_back(ev);
    }
}

void InputCollector::frame(float dt, float screenW, float screenH, InputFrame& in) {
    if (!keyInit_) {
        std::fill(std::begin(keyDuration_), std::end(keyDuration_), -1.0f);
        keyInit_ = true;
    }
    bool clicked[(int)MouseButton::Count] = {}, released[(int)MouseButton::Count] = {};
    bool dbl[(int)MouseButton::Count] = {};
    bool keyChanged[(int)Key::Count] = {};
    bool anyKeyChanged = false, moved = false, wheeled = false, typedText = false;
    unsigned buttonChanged = 0;
    float wheel = 0;
    in.typed.clear();

    // Consume events in order, stopping where ImGui's trickling would, so a
    // change that would be undone within this frame waits for the next.
    size_t used = 0;
    for (; used < queue_.size(); used++) {
        const Event& e = queue_[used];
        bool stop = false;
        switch (e.type) {
        case Ev::Pos:
            if (buttonChanged || wheeled) { stop = true; break; }
            x_ = e.x; y_ = e.y;
            moved = true;
            break;
        case Ev::Button: {
            const unsigned bit = 1u << e.index;
            if ((buttonChanged & bit) || wheeled) { stop = true; break; }
            if (down_[e.index] == e.down) break;   // no change (a release after releaseAll)
            down_[e.index] = e.down;
            buttonChanged |= bit;
            if (e.down) {
                clicked[e.index] = true;
                dbl[e.index] = e.dbl;
                pressX_[e.index] = x_;
                pressY_[e.index] = y_;
                dragMax_[e.index] = 0.0f;
            } else {
                released[e.index] = true;
            }
            break;
        }
        case Ev::Wheel:
            if (moved || buttonChanged) { stop = true; break; }
            wheel += e.y;
            wheeled = true;
            break;
        case Ev::Key:
            if (keyChanged[e.index] || typedText || buttonChanged) { stop = true; break; }
            if (keyDown_[e.index] == e.down) break;
            keyDown_[e.index] = e.down;
            keyChanged[e.index] = true;
            anyKeyChanged = true;
            break;
        case Ev::Char:
            if (buttonChanged || moved || wheeled) { stop = true; break; }
            in.typed.push_back(e.ch);
            typedText = true;
            break;
        case Ev::Mods:
            shift_ = e.shift; ctrl_ = e.ctrl; alt_ = e.alt;
            break;
        }
        if (stop) break;
    }
    queue_.erase(queue_.begin(), queue_.begin() + (std::ptrdiff_t)used);
    (void)anyKeyChanged;

    in.dt = dt;
    in.screenW = screenW;
    in.screenH = screenH;
    in.mouseX = x_;
    in.mouseY = y_;
    const bool bothValid = valid(x_) && valid(prevX_);
    in.mouseDX = bothValid ? x_ - prevX_ : 0.0f;
    in.mouseDY = bothValid ? y_ - prevY_ : 0.0f;
    prevX_ = x_;
    prevY_ = y_;
    in.wheel = wheel;
    in.shift = shift_;
    in.ctrl = ctrl_;
    in.alt = alt_;
    in.uiWantsMouse = false;      // panels are Qt widgets: their input never reaches here
    in.uiWantsKeyboard = false;

    for (int b = 0; b < (int)MouseButton::Count; b++) {
        if (down_[b] && valid(x_)) {
            const float dx = x_ - pressX_[b], dy = y_ - pressY_[b];
            dragMax_[b] = std::max(dragMax_[b], dx * dx + dy * dy);
        }
        in.down[b] = down_[b];
        in.clicked[b] = clicked[b];
        in.released[b] = released[b];
        in.doubleClicked[b] = dbl[b];
        in.pressX[b] = pressX_[b];
        in.pressY[b] = pressY_[b];
        in.dragMaxDistSqr[b] = dragMax_[b];
    }

    for (int k = 0; k < (int)Key::Count; k++) {
        const float prev = keyDuration_[k];
        keyDuration_[k] = keyDown_[k] ? (prev < 0.0f ? 0.0f : prev + dt) : -1.0f;
        const float t = keyDuration_[k];
        bool pressed = false;
        if (t == 0.0f) pressed = true;
        else if (t > 0.0f) pressed = repeats(t - dt, t) > 0;
        in.pressed[(size_t)k] = pressed;
    }
}

} // namespace shitcad
