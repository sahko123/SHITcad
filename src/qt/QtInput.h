#pragma once
#include "ViewportInput.h"

#include <deque>
#include <unordered_map>

class QKeyEvent;
class QMouseEvent;
class QWheelEvent;

namespace shitcad {

// Turns the viewport's Qt events into one InputFrame per frame, with the
// semantics the handlers were written against (Dear ImGui's):
//  - keyPressed() repeats while a key is held: first after 0.275 s, then
//    every 0.05 s, counted from how long the key has been down (OS
//    auto-repeat events are ignored, their text is not);
//  - events are queued and "trickled": a button or key that changes twice
//    within one frame (a quick click) is seen down on one frame and up on the
//    next, so a handler always sees a click with the button held;
//  - dragMaxDistSqr is the furthest the mouse has been from the press;
//  - the mouse position is invalid (-FLT_MAX) once it leaves the view with
//    no button held, and the delta is zero across an invalid position.
// Coordinates are device pixels (logical * devicePixelRatio), the space the
// projection, picking and overlays use.
class InputCollector {
public:
    void mouseMove(QMouseEvent* e, float scale);
    void mouseButton(QMouseEvent* e, float scale, bool down, bool doubleClick);
    void wheel(QWheelEvent* e);
    void key(QKeyEvent* e, bool down);
    void leave(bool buttonHeld);
    // Release every key and modifier, and the buttons too if `buttons`:
    // releases made while another widget or window has the input never
    // arrive here. Keyboard focus moving to a field in the view keeps the
    // mouse (its events still come here); the window deactivating does not.
    void releaseAll(bool buttons = true);

    // Consume queued events up to the trickle limit and fill `in`. `in`
    // keeps the view rect the caller set.
    void frame(float dt, float screenW, float screenH, InputFrame& in);

private:
    enum class Ev { Pos, Button, Wheel, Key, Char, Mods };
    struct Event {
        Ev type;
        float x = 0, y = 0;     // Pos; Wheel uses y
        int index = 0;          // button or Key
        bool down = false;
        bool dbl = false;       // Button: the second press of a double click
        char32_t ch = 0;
        bool shift = false, ctrl = false, alt = false;
    };
    void mods(int qtModifiers);

    std::deque<Event> queue_;
    std::unordered_map<unsigned, int> pressedKeys_;   // scan code -> Key, for the release

    float x_ = -3.4e38f, y_ = -3.4e38f;   // current, invalid while outside
    float prevX_ = -3.4e38f, prevY_ = -3.4e38f;
    bool shift_ = false, ctrl_ = false, alt_ = false;
    bool down_[(int)MouseButton::Count] = {};
    float pressX_[(int)MouseButton::Count] = {}, pressY_[(int)MouseButton::Count] = {};
    float dragMax_[(int)MouseButton::Count] = {};
    bool keyDown_[(int)Key::Count] = {};
    float keyDuration_[(int)Key::Count] = {};   // -1 while up
    bool keyInit_ = false;
};

} // namespace shitcad
