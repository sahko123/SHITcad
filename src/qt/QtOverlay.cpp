#include "QtOverlay.h"

#include <QFont>
#include <QFontMetricsF>
#include <QPainter>
#include <QPainterPath>

#include <cmath>
#include <map>

namespace shitcad {

namespace {

float g_scale = 1.0f;

// Font for text drawn at `scale` times the UI size, in device pixels.
const QFont& fontFor(float scale) {
    static std::map<int, QFont> cache;
    const int px = std::max(1, (int)std::lround(15.0f * g_scale * scale));
    auto it = cache.find(px);
    if (it == cache.end()) {
        QFont f("Segoe UI");
        f.setPixelSize(px);
        it = cache.emplace(px, f).first;
    }
    return it->second;
}

QColor colour(Color32 c) {
    return QColor((int)(c & 0xFF), (int)((c >> 8) & 0xFF), (int)((c >> 16) & 0xFF), (int)((c >> 24) & 0xFF));
}

constexpr int kTextFlags = Qt::AlignLeft | Qt::AlignTop | Qt::TextDontClip;

} // namespace

void setOverlayScale(float devicePixelRatio) { g_scale = devicePixelRatio; }

OvVec2 overlayMeasure(const char* text, float scale) {
    const QFontMetricsF fm(fontFor(scale));
    const QRectF r = fm.boundingRect(QRectF(0, 0, 1e6, 1e6), kTextFlags, QString::fromUtf8(text));
    return {(float)r.width(), (float)r.height()};
}

void drawOverlay(QPainter& p, const Overlay2D& overlay, float devicePixelRatio) {
    p.save();
    p.scale(1.0 / devicePixelRatio, 1.0 / devicePixelRatio);   // recorded in device pixels
    p.setRenderHint(QPainter::Antialiasing);
    p.setRenderHint(QPainter::TextAntialiasing);
    auto pt = [](OvVec2 v) { return QPointF(v.x, v.y); };
    for (const auto& c : overlay.commands()) {
        const QColor col = colour(c.col);
        switch (c.kind) {
        case Overlay2D::Kind::Line:
            p.setPen(QPen(col, c.thickness, Qt::SolidLine, Qt::FlatCap));
            p.drawLine(pt(c.a), pt(c.b));
            break;
        case Overlay2D::Kind::Rect:
            p.setPen(QPen(col, c.thickness));
            p.setBrush(Qt::NoBrush);
            p.drawRoundedRect(QRectF(pt(c.a), pt(c.b)).normalized(), c.rounding, c.rounding);
            break;
        case Overlay2D::Kind::RectFilled:
            p.setPen(Qt::NoPen);
            p.setBrush(col);
            p.drawRoundedRect(QRectF(pt(c.a), pt(c.b)).normalized(), c.rounding, c.rounding);
            break;
        case Overlay2D::Kind::TriangleFilled: {
            p.setPen(Qt::NoPen);
            p.setBrush(col);
            const QPointF tri[3] = {pt(c.a), pt(c.b), pt(c.c)};
            p.drawPolygon(tri, 3);
            break;
        }
        case Overlay2D::Kind::Circle:
            p.setPen(QPen(col, c.thickness));
            p.setBrush(Qt::NoBrush);
            p.drawEllipse(pt(c.a), c.rounding, c.rounding);
            break;
        case Overlay2D::Kind::CircleFilled:
            p.setPen(Qt::NoPen);
            p.setBrush(col);
            p.drawEllipse(pt(c.a), c.rounding, c.rounding);
            break;
        case Overlay2D::Kind::Text:
            p.setFont(fontFor(c.textScale));
            p.setPen(col);
            p.drawText(QRectF(c.a.x, c.a.y, 1e6, 1e6), kTextFlags, QString::fromUtf8(c.text.c_str()));
            break;
        }
    }
    p.restore();
}

} // namespace shitcad
