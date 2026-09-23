#pragma once
#include "Overlay2D.h"

class QPainter;

namespace shitcad {

// Draws App's recorded Overlay2D list with QPainter, and measures overlay
// text with the same font, so label layout and hit rectangles agree with
// what is drawn. Coordinates are device pixels, as recorded.
//
// The UI font is Segoe UI at 15 px times the display scale (the size ImGui
// used). Call setOverlayScale before the first frame and when the scale
// changes; overlayMeasure is the Overlay2D::MeasureFn to give App.
void setOverlayScale(float devicePixelRatio);
OvVec2 overlayMeasure(const char* text, float scale);
void drawOverlay(QPainter& p, const Overlay2D& overlay, float devicePixelRatio);

} // namespace shitcad
