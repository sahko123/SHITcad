#pragma once
#include "App.h"

#include <QWidget>

#undef near
#undef far

namespace shitcad {

// Qt front end for TimelineModel: a strip over the bottom of the viewport
// (not a dock, so the view does not resize when the first feature appears),
// with a button per feature and the playhead. Painted by hand: click
// selects, double-click edits, right-click renames / suppresses / deletes,
// and the playhead drags between features. The wheel scrolls a long history.
class Timeline : public QWidget {
    Q_OBJECT
public:
    Timeline(App& app, QWidget* viewport);
    void refresh();

protected:
    void paintEvent(QPaintEvent* e) override;
    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;
    void mouseDoubleClickEvent(QMouseEvent* e) override;
    void wheelEvent(QWheelEvent* e) override;
    void leaveEvent(QEvent* e) override;
    bool event(QEvent* e) override;   // error tooltips

private:
    void layoutItems();
    int itemAt(int x) const;          // -1 when not on a button
    bool onPlayhead(int x) const;
    int nearestEdge(int x) const;     // playhead position for a mouse x
    int headX() const;
    void contextMenu(int item, const QPoint& globalPos);

    App& app_;
    QWidget* viewport_;
    App::TimelineModel model_;
    std::vector<QRect> rects_;        // one per item, in widget coordinates
    int scroll_ = 0;
    int hovered_ = -1, pressed_ = -1;
    bool headHovered_ = false, dragging_ = false;
};

} // namespace shitcad
