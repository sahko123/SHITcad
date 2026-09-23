#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace shitcad {

// 32-bit colour, packed as R | G << 8 | B << 16 | A << 24.
using Color32 = uint32_t;
constexpr Color32 rgba32(int r, int g, int b, int a) {
    return (Color32)(r & 0xFF) | ((Color32)(g & 0xFF) << 8) |
           ((Color32)(b & 0xFF) << 16) | ((Color32)(a & 0xFF) << 24);
}

struct OvVec2 {
    float x = 0, y = 0;
    OvVec2() = default;
    OvVec2(float x_, float y_) : x(x_), y(y_) {}
};

// Screen-space drawing over the 3D view: dimension labels, handles, rulers,
// readouts. Commands are recorded while the frame is built - including from
// inside the GL pass - and the host draws the whole list once, after the 3D
// scene and before the UI, in the order they were recorded. Coordinates are
// window pixels, the same space as InputFrame.
//
// Text is measured immediately (layout needs it) through a function the host
// provides, so labels and their hit rectangles always agree with what is drawn.
class Overlay2D {
public:
    enum class Kind : uint8_t { Line, Rect, RectFilled, TriangleFilled, Circle, CircleFilled, Text };
    struct Cmd {
        Kind kind;
        OvVec2 a, b, c;           // endpoints / corners / centre (a) / text position (a)
        Color32 col = 0;
        float thickness = 1.0f;
        float rounding = 0.0f;    // rects; circle radius for circles
        int segments = 0;         // circles: 0 = automatic
        float textScale = 1.0f;   // text: relative to the UI font size
        std::string text;
    };

    // Returns the size of `text` drawn at `scale` times the UI font size.
    using MeasureFn = OvVec2 (*)(const char* text, float scale);
    void setMeasure(MeasureFn fn) { measure_ = fn; }

    void clear() { cmds_.clear(); }
    const std::vector<Cmd>& commands() const { return cmds_; }

    void addLine(OvVec2 a, OvVec2 b, Color32 col, float thickness = 1.0f) {
        Cmd m{Kind::Line, a, b, {}, col}; m.thickness = thickness; cmds_.push_back(std::move(m));
    }
    void addRect(OvVec2 a, OvVec2 b, Color32 col, float rounding = 0.0f, int /*flags*/ = 0, float thickness = 1.0f) {
        Cmd m{Kind::Rect, a, b, {}, col}; m.rounding = rounding; m.thickness = thickness; cmds_.push_back(std::move(m));
    }
    void addRectFilled(OvVec2 a, OvVec2 b, Color32 col, float rounding = 0.0f) {
        Cmd m{Kind::RectFilled, a, b, {}, col}; m.rounding = rounding; cmds_.push_back(std::move(m));
    }
    void addTriangleFilled(OvVec2 a, OvVec2 b, OvVec2 c, Color32 col) {
        cmds_.push_back(Cmd{Kind::TriangleFilled, a, b, c, col});
    }
    void addCircle(OvVec2 centre, float radius, Color32 col, int segments = 0, float thickness = 1.0f) {
        Cmd m{Kind::Circle, centre, {}, {}, col}; m.rounding = radius; m.segments = segments; m.thickness = thickness;
        cmds_.push_back(std::move(m));
    }
    void addCircleFilled(OvVec2 centre, float radius, Color32 col, int segments = 0) {
        Cmd m{Kind::CircleFilled, centre, {}, {}, col}; m.rounding = radius; m.segments = segments;
        cmds_.push_back(std::move(m));
    }
    void addText(OvVec2 pos, Color32 col, const char* text, float scale = 1.0f) {
        Cmd m{Kind::Text, pos, {}, {}, col}; m.textScale = scale; m.text = text; cmds_.push_back(std::move(m));
    }

    OvVec2 textSize(const char* text, float scale = 1.0f) const {
        if (measure_) return measure_(text, scale);
        // No host (tests): a rough monospace estimate.
        size_t n = 0;
        for (const char* p = text; *p; p++) n += ((unsigned char)*p & 0xC0) != 0x80; // count UTF-8 code points
        return {7.0f * scale * (float)n, 15.0f * scale};
    }

private:
    std::vector<Cmd> cmds_;
    MeasureFn measure_ = nullptr;
};

} // namespace shitcad
