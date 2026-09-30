#include "Timeline.h"

#include <QHBoxLayout>
#include <QHelpEvent>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QMouseEvent>
#include <QPainter>
#include <QToolTip>
#include <QWheelEvent>
#include <QWidgetAction>

#include <algorithm>
#include <climits>
#include <cmath>

namespace shitcad {

namespace {
constexpr int kPad = 8;        // around the strip and between buttons
constexpr int kTextPad = 8;    // inside a button, each side
constexpr int kHandleW = 6;    // playhead bar width

QColor scaled(const float rgb[3], float k) {
    return QColor::fromRgbF(std::min(rgb[0] * k, 1.0f), std::min(rgb[1] * k, 1.0f), std::min(rgb[2] * k, 1.0f));
}

bool sameModel(const App::TimelineModel& a, const App::TimelineModel& b) {
    if (a.visible != b.visible || a.playhead != b.playhead || a.dragging != b.dragging ||
        a.items.size() != b.items.size())
        return false;
    for (size_t i = 0; i < a.items.size(); i++) {
        const auto& x = a.items[i];
        const auto& y = b.items[i];
        if (x.id != y.id || x.name != y.name || x.grayed != y.grayed || x.error != y.error ||
            x.selected != y.selected || x.suppressed != y.suppressed || x.errorMsg != y.errorMsg ||
            !std::equal(x.rgb, x.rgb + 3, y.rgb))
            return false;
    }
    return true;
}
}

Timeline::Timeline(App& app, QWidget* viewport) : QWidget(viewport), app_(app), viewport_(viewport) {
    setAutoFillBackground(true);
    setMouseTracking(true);
    setFocusPolicy(Qt::NoFocus);
    hide();
}

void Timeline::refresh() {
    App::TimelineModel m = app_.timelineModel();
    const int h = fontMetrics().height() + 8 + kPad * 2;
    App* a = &app_;
    const float hostH = m.visible ? (float)h : 0.0f;
    app_.post([a, hostH] { a->setHostTimelineHeight(hostH); });

    if (!m.visible) {
        if (!isHidden()) hide();
        dragging_ = false;
        return;
    }
    const QRect want(0, viewport_->height() - h, viewport_->width(), h);
    const bool moved = geometry() != want;
    if (moved) setGeometry(want);
    if (!moved && sameModel(m, model_) && !isHidden()) return;
    model_ = std::move(m);
    layoutItems();
    if (isHidden()) { show(); raise(); }
    update();
}

void Timeline::layoutItems() {
    rects_.clear();
    const QFontMetrics fm = fontMetrics();
    const int barH = fm.height() + 8;
    int x = kPad - scroll_;
    for (const auto& item : model_.items) {
        const int w = fm.horizontalAdvance(QString::fromStdString(item.name)) + kTextPad * 2;
        rects_.emplace_back(x, (height() - barH) / 2, w, barH);
        x += w + kPad;
    }
    // Keep the scroll inside the content.
    const int contentW = x + scroll_;   // unscrolled right edge, padding included
    const int maxScroll = std::max(0, contentW - width());
    if (scroll_ > maxScroll) {
        scroll_ = maxScroll;
        layoutItems();
    }
}

int Timeline::itemAt(int x) const {
    for (int i = 0; i < (int)rects_.size(); i++)
        if (x >= rects_[i].left() && x <= rects_[i].right()) return i;
    return -1;
}

int Timeline::headX() const {
    if (model_.playhead < 0 || model_.playhead >= (int)rects_.size()) return -100;
    return rects_[model_.playhead].right() + 1 + 2;
}

bool Timeline::onPlayhead(int x) const {
    return std::abs(x - headX()) <= kHandleW / 2 + 2;
}

// Snap to the nearest feature's right edge; past the last one is the end (-1).
int Timeline::nearestEdge(int x) const {
    const int n = (int)rects_.size();
    int best = n - 1;
    int bestDist = INT_MAX;
    for (int i = 0; i < n; i++) {
        const int d = std::abs(x - (rects_[i].right() + 1));
        if (d < bestDist) { bestDist = d; best = i; }
    }
    return best >= n - 1 ? -1 : best;
}

void Timeline::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    for (int i = 0; i < (int)rects_.size(); i++) {
        const auto& item = model_.items[i];
        const QRectF r = rects_[i];
        const float k = (i == pressed_) ? 0.8f : (i == hovered_ ? 1.2f : 1.0f);
        p.setPen(Qt::NoPen);
        p.setBrush(scaled(item.rgb, k));
        p.drawRoundedRect(r, 4, 4);
        if (item.error && !item.grayed) {
            p.setPen(QPen(QColor(255, 0, 0), 2));
            p.setBrush(Qt::NoBrush);
            p.drawRoundedRect(r.adjusted(1, 1, -1, -1), 4, 4);
        }
        if (item.selected) {
            p.setPen(QPen(QColor(0, 200, 255), 3));
            p.setBrush(Qt::NoBrush);
            p.drawRoundedRect(r.adjusted(-2, -2, 2, 2), 3, 3);
        }
        p.setPen(palette().color(QPalette::WindowText));
        p.drawText(r, Qt::AlignCenter, QString::fromStdString(item.name));
    }

    // Playhead: a bar after the last feature in effect, with a grip triangle
    const int hx = headX();
    if (!rects_.empty() && hx > -100) {
        const QRect bar = rects_[0];
        const QColor col = (headHovered_ || dragging_) ? QColor(255, 60, 40) : QColor(220, 50, 30);
        p.setPen(Qt::NoPen);
        p.setBrush(col);
        p.drawRoundedRect(QRectF(hx - kHandleW / 2.0, bar.top(), kHandleW, bar.height()), 2, 2);
        const QPointF tri[3] = {{hx - 4.0, (double)bar.top()}, {hx + 4.0, (double)bar.top()},
                                {(double)hx, bar.top() + 4.8}};
        p.drawPolygon(tri, 3);
    }
}

void Timeline::mousePressEvent(QMouseEvent* e) {
    App* a = &app_;
    const int x = (int)e->position().x();
    if (e->button() == Qt::LeftButton && onPlayhead(x)) {
        dragging_ = true;
        a->post([a] { a->beginPlayheadDrag(); });
        update();
        return;
    }
    const int i = itemAt(x);
    if (i < 0) return;
    const uint32_t id = model_.items[i].id;
    if (e->button() == Qt::LeftButton) {
        pressed_ = i;
        a->post([a, id] { a->selectFeature(id); });
        update();
    } else if (e->button() == Qt::RightButton) {
        contextMenu(i, e->globalPosition().toPoint());
    }
}

void Timeline::mouseMoveEvent(QMouseEvent* e) {
    const int x = (int)e->position().x();
    if (dragging_) {
        App* a = &app_;
        const int pos = nearestEdge(x);
        a->post([a, pos] { a->movePlayhead(pos); });
        return;
    }
    const int hov = itemAt(x);
    const bool head = onPlayhead(x);
    if (hov != hovered_ || head != headHovered_) {
        hovered_ = hov;
        headHovered_ = head;
        setCursor(head ? Qt::SizeHorCursor : Qt::ArrowCursor);
        update();
    }
}

void Timeline::mouseReleaseEvent(QMouseEvent* e) {
    if (e->button() != Qt::LeftButton) return;
    if (dragging_) {
        dragging_ = false;
        App* a = &app_;
        a->post([a] { a->endPlayheadDrag(); });
    }
    pressed_ = -1;
    update();
}

void Timeline::mouseDoubleClickEvent(QMouseEvent* e) {
    if (e->button() != Qt::LeftButton) return;
    const int i = itemAt((int)e->position().x());
    if (i < 0 || onPlayhead((int)e->position().x())) return;
    App* a = &app_;
    const uint32_t id = model_.items[i].id;
    a->post([a, id] { a->editFeature(id); });
}

void Timeline::wheelEvent(QWheelEvent* e) {
    const int delta = e->angleDelta().y() != 0 ? e->angleDelta().y() : e->angleDelta().x();
    scroll_ = std::max(0, scroll_ - delta / 2);
    layoutItems();
    update();
}

void Timeline::leaveEvent(QEvent*) {
    if (hovered_ != -1 || headHovered_) {
        hovered_ = -1;
        headHovered_ = false;
        unsetCursor();
        update();
    }
}

bool Timeline::event(QEvent* e) {
    if (e->type() == QEvent::ToolTip) {
        auto* help = static_cast<QHelpEvent*>(e);
        const int i = itemAt(help->pos().x());
        if (i >= 0 && model_.items[i].error)
            QToolTip::showText(help->globalPos(), QString::fromStdString(model_.items[i].errorMsg), this);
        else
            QToolTip::hideText();
        return true;
    }
    return QWidget::event(e);
}

void Timeline::contextMenu(int i, const QPoint& globalPos) {
    const App::TimelineModel::Item item = model_.items[i];
    App* a = &app_;
    const uint32_t id = item.id;

    QMenu menu(this);
    // Name field: Enter renames
    auto* nameRow = new QWidget(&menu);
    auto* row = new QHBoxLayout(nameRow);
    row->setContentsMargins(8, 4, 8, 4);
    auto* name = new QLineEdit(QString::fromStdString(item.name), nameRow);
    name->setMinimumWidth(140);
    row->addWidget(name);
    row->addWidget(new QLabel("Name", nameRow));
    auto* nameAction = new QWidgetAction(&menu);
    nameAction->setDefaultWidget(nameRow);
    menu.addAction(nameAction);
    connect(name, &QLineEdit::returnPressed, &menu, [&menu, a, id, name] {
        const std::string s = name->text().toUtf8().toStdString();
        a->post([a, id, s] { a->renameFeature(id, s); });
        menu.close();
    });
    menu.addSeparator();
    if (item.canPlace)
        menu.addAction("Rotate / Move...", [a, id] { a->post([a, id] { a->editFeature(id); }); });
    if (item.canReplaceFile)
        menu.addAction("Replace file...", [a, id] { a->post([a, id] { a->replaceImportFile(id); }); });
    const bool suppress = !item.suppressed;
    menu.addAction(suppress ? "Suppress" : "Unsuppress",
                   [a, id, suppress] { a->post([a, id, suppress] { a->setFeatureSuppressed(id, suppress); }); });
    menu.addAction("Delete", [a, id] { a->post([a, id] { a->deleteFeature(id); }); });
    name->setFocus();
    name->selectAll();
    menu.exec(globalPos);
}

} // namespace shitcad
