#include "ObjectTree.h"

#include <QCloseEvent>
#include <QCursor>
#include <QPushButton>
#include <QSignalBlocker>
#include <QStyle>
#include <QStyleOptionViewItem>
#include <QTreeWidget>
#include <QVBoxLayout>

namespace shitcad {

namespace {
constexpr int kKeyRole = Qt::UserRole;          // ObjectTreeModel::Row::key
constexpr int kIndexRole = Qt::UserRole + 1;    // Row::index
constexpr int kSectionRole = Qt::UserRole + 2;  // which section a heading is
}

ObjectTree::ObjectTree(App& app, QWidget* parent) : QDockWidget("Object Tree", parent), app_(app) {
    setObjectName("objectTree");   // saveState() keys docks by name
    setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetMovable);
    App* a = &app_;

    auto* body = new QWidget(this);
    auto* layout = new QVBoxLayout(body);
    layout->setContentsMargins(4, 4, 4, 4);
    tree_ = new QTreeWidget(body);
    tree_->setHeaderHidden(true);
    tree_->setColumnCount(1);
    layout->addWidget(tree_);
    auto* addPlane = new QPushButton("+ Add Plane", body);
    layout->addWidget(addPlane);
    setWidget(body);

    const char* titles[] = {"Reference Planes", "Sketches", "Bodies"};
    for (int s = 0; s < 3; s++) {
        sections_[s] = new QTreeWidgetItem(tree_, QStringList(titles[s]));
        sections_[s]->setFlags(Qt::ItemIsEnabled);
        sections_[s]->setData(0, kSectionRole, s);
        sections_[s]->setExpanded(true);
    }

    // Only user edits arrive here: refresh() blocks the tree's signals.
    connect(tree_, &QTreeWidget::itemChanged, this, [a](QTreeWidgetItem* it, int) {
        if (!it->parent()) return;
        const int section = it->parent()->data(0, kSectionRole).toInt();
        const int index = it->data(0, kIndexRole).toInt();
        const bool on = it->checkState(0) == Qt::Checked;
        a->post([a, section, index, on] {
            if (section == Planes) a->setPlaneVisible(index, on);
            else if (section == Sketches) a->setSketchVisible(index, on);
            else a->setBodyVisible(index, on);
        });
    });
    connect(tree_, &QTreeWidget::itemDoubleClicked, this, [this, a](QTreeWidgetItem* it, int) {
        if (!it->parent() || it->parent()->data(0, kSectionRole).toInt() == Bodies) return;
        // A double click on the check box only toggles it twice, as in ImGui.
        QStyleOptionViewItem opt;
        opt.initFrom(tree_);
        opt.rect = tree_->visualItemRect(it);
        opt.features |= QStyleOptionViewItem::HasCheckIndicator;
        const QRect check = tree_->style()->subElementRect(QStyle::SE_ItemViewItemCheckIndicator, &opt, tree_);
        if (check.contains(tree_->viewport()->mapFromGlobal(QCursor::pos()))) return;
        const int index = it->data(0, kIndexRole).toInt();
        a->post([a, index] { a->editPlaneSketch(index); });
    });
    connect(addPlane, &QPushButton::clicked, this, [a] { a->post([a] { a->openAddPlaneDialog(); }); });
}

// Brings the section's children in line with rows, matching by key: a row
// already shown is updated (and moved if its position changed), a new one
// is inserted, and rows no longer in the model are deleted.
void ObjectTree::sync(QTreeWidgetItem* section, const std::vector<App::ObjectTreeModel::Row>& rows) {
    for (int i = 0; i < (int)rows.size(); i++) {
        const auto& r = rows[i];
        QTreeWidgetItem* it = nullptr;
        for (int j = i; j < section->childCount(); j++) {
            if (section->child(j)->data(0, kKeyRole).toULongLong() != r.key) continue;
            it = section->child(j);
            if (j != i) { section->takeChild(j); section->insertChild(i, it); }
            break;
        }
        if (!it) {
            it = new QTreeWidgetItem;
            it->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsUserCheckable);
            it->setData(0, kKeyRole, (qulonglong)r.key);
            section->insertChild(i, it);
        }
        it->setData(0, kIndexRole, r.index);
        const QString label = QString::fromStdString(r.label);
        if (it->text(0) != label) it->setText(0, label);
        const Qt::CheckState cs = r.visible ? Qt::Checked : Qt::Unchecked;
        if (it->checkState(0) != cs) it->setCheckState(0, cs);
    }
    while (section->childCount() > (int)rows.size())
        delete section->takeChild(section->childCount() - 1);
}

void ObjectTree::refresh() {
    const App::ObjectTreeModel m = app_.objectTreeModel();
    // isHidden, not isVisible: a minimised window's docks are not visible.
    if (m.open == isHidden()) setVisible(m.open);
    if (!m.open) return;

    const QSignalBlocker block(tree_);
    sync(sections_[Planes], m.planes);
    sync(sections_[Sketches], m.sketches);
    sync(sections_[Bodies], m.bodies);
    // As in the ImGui tree, empty Sketches and Bodies sections are left out.
    sections_[Sketches]->setHidden(m.sketches.empty());
    sections_[Bodies]->setHidden(m.bodies.empty());
}

void ObjectTree::closeEvent(QCloseEvent* e) {
    App* a = &app_;
    app_.post([a] { a->setObjectTreeOpen(false); });
    QDockWidget::closeEvent(e);
}

} // namespace shitcad
