#pragma once
#include "App.h"

#include <QDockWidget>

#undef near
#undef far

class QTreeWidget;
class QTreeWidgetItem;

namespace shitcad {

// Qt front end for ObjectTreeModel:
// reference planes, sketches and bodies with visibility checks, in a dock.
// Rows are keyed and updated in place, so the current item, expansion and
// scroll position survive planes and bodies being added or removed.
class ObjectTree : public QDockWidget {
    Q_OBJECT
public:
    ObjectTree(App& app, QWidget* parent = nullptr);
    void refresh();

protected:
    void closeEvent(QCloseEvent* e) override;

private:
    enum Section { Planes, Sketches, Bodies };
    void sync(QTreeWidgetItem* section, const std::vector<App::ObjectTreeModel::Row>& rows);

    App& app_;
    QTreeWidget* tree_ = nullptr;
    QTreeWidgetItem* sections_[3] = {};
};

} // namespace shitcad
