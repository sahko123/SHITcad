#pragma once
#include <bitset>
#include <cstdint>
#include <string>

namespace shitcad {

// Keys the viewport handlers react to. Digits are contiguous so handlers can
// loop over them (Num0 + i, Keypad0 + i).
enum class Key : uint8_t {
    A, C, D, E, F, L, N, O, P, R, S, T, V, Y, Z,
    Num0, Num1, Num2, Num3, Num4, Num5, Num6, Num7, Num8, Num9,
    Keypad0, Keypad1, Keypad2, Keypad3, Keypad4, Keypad5, Keypad6, Keypad7, Keypad8, Keypad9,
    KeypadDecimal, KeypadEnter, Period, Enter, Escape, Delete, Backspace,
    Count
};

enum class MouseButton : uint8_t { Left, Right, Middle, Count };

inline Key digitKey(int d) { return (Key)((int)Key::Num0 + d); }
inline Key keypadDigitKey(int d) { return (Key)((int)Key::Keypad0 + d); }

// One frame of input for the 3D viewport, independent of the GUI toolkit.
// The host fills it once per frame, before any handler runs; handlers read
// only this. The semantics are Dear ImGui's, which the UI used first and the
// handlers were written against (src/qt/QtInput.cpp reproduces them):
//  - keyPressed() includes auto-repeat (0.275 s, then every 0.05 s).
//  - clicked is true on every press, including the second press of a double click.
//  - dragging() compares the *furthest* the mouse has been from the press
//    position, not the current distance, so a drag stays a drag.
//  - typed holds the characters typed this frame.
// uiWantsMouse/uiWantsKeyboard are false with the Qt host: its panels are
// separate widgets whose input never reaches the viewport.
// Coordinates are in the window space the projection and picking code uses.
struct InputFrame {
    float mouseX = 0, mouseY = 0;
    float mouseDX = 0, mouseDY = 0;
    float wheel = 0;
    float dt = 0;

    // Where the 3D view takes input: below the toolbar, the full window width.
    // Panels drawn over it are excluded through uiWantsMouse, not this rect.
    float viewX = 0, viewY = 0, viewW = 0, viewH = 0;
    // Size of the whole space the coordinates are in (the window), which is
    // what projection and picking are computed against.
    float screenW = 0, screenH = 0;

    bool shift = false, ctrl = false, alt = false;
    bool uiWantsMouse = false;     // a panel or widget has the mouse
    bool uiWantsKeyboard = false;  // a text field or widget has the keyboard

    bool down[(int)MouseButton::Count] = {};
    bool clicked[(int)MouseButton::Count] = {};
    bool released[(int)MouseButton::Count] = {};
    bool doubleClicked[(int)MouseButton::Count] = {};
    float pressX[(int)MouseButton::Count] = {};  // where the last press happened
    float pressY[(int)MouseButton::Count] = {};
    float dragMaxDistSqr[(int)MouseButton::Count] = {};

    std::bitset<(size_t)Key::Count> pressed;
    std::u32string typed;

    bool keyPressed(Key k) const { return pressed[(size_t)k]; }
    bool mouseDown(MouseButton b) const { return down[(int)b]; }
    bool mouseClicked(MouseButton b) const { return clicked[(int)b]; }
    bool mouseReleased(MouseButton b) const { return released[(int)b]; }
    bool mouseDoubleClicked(MouseButton b) const { return doubleClicked[(int)b]; }
    bool anyMouseDown() const { return down[0] || down[1] || down[2]; }

    // Held, and has moved at least `threshold` pixels from the press since.
    bool dragging(MouseButton b, float threshold) const {
        return down[(int)b] && dragMaxDistSqr[(int)b] >= threshold * threshold;
    }

    // Mouse inside the 3D view's input rect.
    bool hovered() const {
        return mouseX >= viewX && mouseX < viewX + viewW &&
               mouseY >= viewY && mouseY < viewY + viewH;
    }
};

} // namespace shitcad
