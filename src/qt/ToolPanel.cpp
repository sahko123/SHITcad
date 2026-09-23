#include "ToolPanel.h"

#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QStackedWidget>
#include <QStandardItemModel>
#include <QVBoxLayout>

namespace shitcad {

namespace {

// Buttons never take the keyboard: it stays with the viewport, or the field.
QPushButton* button(const QString& text, QWidget* parent) {
    auto* b = new QPushButton(text, parent);
    b->setFocusPolicy(Qt::NoFocus);
    return b;
}

QFrame* separator(QWidget* parent) {
    auto* line = new QFrame(parent);
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
    return line;
}

void clearLayout(QLayout* layout) {
    while (QLayoutItem* item = layout->takeAt(0)) {
        delete item->widget();
        delete item;
    }
}

void setCombo(QComboBox* combo, int index) {
    if (combo->currentIndex() == index) return;
    const QSignalBlocker block(combo);
    combo->setCurrentIndex(index);
}

void setLabel(QLabel* label, const QString& text) {
    if (label->text() != text) label->setText(text);
}

// OK and Cancel side by side.
QHBoxLayout* okCancel(QPushButton* ok, QPushButton* cancel) {
    auto* row = new QHBoxLayout;
    row->addWidget(ok);
    row->addWidget(cancel);
    return row;
}

} // namespace

ToolPanel::ToolPanel(App& app, QWidget* viewport) : QFrame(viewport), app_(app), viewport_(viewport) {
    setFrameShape(QFrame::StyledPanel);
    setAutoFillBackground(true);
    setFocusPolicy(Qt::NoFocus);
    setFixedWidth(240);

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(10, 8, 10, 10);
    title_ = new QLabel(this);
    QFont bold = title_->font();
    bold.setBold(true);
    title_->setFont(bold);
    root->addWidget(title_);
    pages_ = new QStackedWidget(this);
    pages_->addWidget(buildExtrude());
    pages_->addWidget(buildRevolve());
    pages_->addWidget(buildLoft());
    pages_->addWidget(buildBoolean());
    root->addWidget(pages_);
    hide();
}

QLineEdit* ToolPanel::valueField(QWidget* parent, void (App::*apply)(const std::string&)) {
    auto* field = new QLineEdit(parent);
    App* a = &app_;
    connect(field, &QLineEdit::editingFinished, this, [this, a, field, apply] {
        const std::string text = field->text().toUtf8().toStdString();
        a->post([a, apply, text] { (a->*apply)(text); });
        if (field->hasFocus()) viewport_->setFocus();   // Enter (focus loss also lands here)
    });
    return field;
}

void ToolPanel::setField(QLineEdit* field, const std::string& text) {
    if (field->hasFocus()) return;   // never overwrite what is being typed
    const QString t = QString::fromStdString(text);
    if (field->text() == t) return;
    const QSignalBlocker block(field);
    field->setText(t);
    field->setCursorPosition(0);
}

// ---- pages ---------------------------------------------------------------

QWidget* ToolPanel::buildExtrude() {
    auto* page = new QWidget(this);
    auto* col = new QVBoxLayout(page);
    col->setContentsMargins(0, 0, 0, 0);
    App* a = &app_;

    col->addWidget(new QLabel("Operation:", page));
    exOp_ = new QComboBox(page);
    exOp_->addItems({"New Body", "Cut"});
    col->addWidget(exOp_);
    col->addWidget(new QLabel("Distance:", page));
    exDist_ = valueField(page, &App::setExtrudeDistanceText);
    col->addWidget(exDist_);
    col->addWidget(new QLabel("Direction:", page));
    exDir_ = new QComboBox(page);
    exDir_->addItems({"One Side", "Other Side", "Both Sides", "Symmetric"});
    col->addWidget(exDir_);
    col->addWidget(new QLabel("Offset:", page));
    exOffset_ = valueField(page, &App::setExtrudeOffsetText);
    col->addWidget(exOffset_);
    col->addWidget(separator(page));
    exProfiles_ = new QLabel(page);
    col->addWidget(exProfiles_);
    col->addWidget(separator(page));
    exOk_ = button("OK [Enter]", page);
    auto* cancel = button("Cancel [Esc]", page);
    col->addLayout(okCancel(exOk_, cancel));

    connect(exOp_, &QComboBox::activated, this, [a](int i) { a->post([a, i] { a->setExtrudeOperation(i); }); });
    connect(exDir_, &QComboBox::activated, this, [a](int i) { a->post([a, i] { a->setExtrudeDirection(i); }); });
    connect(exOk_, &QPushButton::clicked, this, [a] { a->post([a] { a->commitExtrude(); }); });
    connect(cancel, &QPushButton::clicked, this, [a] { a->post([a] { a->cancelExtrude(); }); });
    return page;
}

QWidget* ToolPanel::buildRevolve() {
    auto* page = new QWidget(this);
    auto* col = new QVBoxLayout(page);
    col->setContentsMargins(0, 0, 0, 0);
    App* a = &app_;

    col->addWidget(new QLabel("Operation:", page));
    rvOp_ = new QComboBox(page);
    rvOp_->addItems({"New Body", "Cut"});
    col->addWidget(rvOp_);
    col->addWidget(new QLabel("Angle (deg):", page));
    rvAngle_ = valueField(page, &App::setRevolveAngleText);
    col->addWidget(rvAngle_);
    col->addWidget(separator(page));
    rvAxis_ = new QLabel(page);
    col->addWidget(rvAxis_);
    rvChangeAxis_ = button("Change Axis", page);
    col->addWidget(rvChangeAxis_);
    rvPickHint_ = new QLabel("Click a line for axis", page);
    rvPickHint_->setStyleSheet("color: #b08000;");
    col->addWidget(rvPickHint_);
    rvSelectAxis_ = button("Select Axis", page);
    col->addWidget(rvSelectAxis_);
    col->addWidget(separator(page));
    rvProfiles_ = new QLabel(page);
    col->addWidget(rvProfiles_);
    rvSelectProfiles_ = button("Select Profiles", page);
    col->addWidget(rvSelectProfiles_);
    col->addWidget(separator(page));
    rvOk_ = button("OK [Enter]", page);
    auto* cancel = button("Cancel [Esc]", page);
    col->addLayout(okCancel(rvOk_, cancel));

    connect(rvOp_, &QComboBox::activated, this, [a](int i) { a->post([a, i] { a->setRevolveOperation(i); }); });
    connect(rvChangeAxis_, &QPushButton::clicked, this, [a] { a->post([a] { a->pickRevolveAxis(true); }); });
    connect(rvSelectAxis_, &QPushButton::clicked, this, [a] { a->post([a] { a->pickRevolveAxis(false); }); });
    connect(rvSelectProfiles_, &QPushButton::clicked, this, [a] { a->post([a] { a->pickRevolveProfiles(); }); });
    connect(rvOk_, &QPushButton::clicked, this, [a] { a->post([a] { a->commitRevolve(); }); });
    connect(cancel, &QPushButton::clicked, this, [a] { a->post([a] { a->cancelRevolve(); }); });
    return page;
}

QWidget* ToolPanel::buildLoft() {
    auto* page = new QWidget(this);
    auto* col = new QVBoxLayout(page);
    col->setContentsMargins(0, 0, 0, 0);
    App* a = &app_;

    auto* intro = new QLabel("Select profiles on different sketch planes to loft between.", page);
    intro->setWordWrap(true);
    col->addWidget(intro);
    col->addWidget(separator(page));
    lfSections_ = new QWidget(page);
    new QVBoxLayout(lfSections_);
    lfSections_->layout()->setContentsMargins(0, 0, 0, 0);
    col->addWidget(lfSections_);
    col->addWidget(separator(page));
    col->addWidget(new QLabel("Add section:", page));
    lfCandidates_ = new QWidget(page);
    new QVBoxLayout(lfCandidates_);
    lfCandidates_->layout()->setContentsMargins(0, 0, 0, 0);
    col->addWidget(lfCandidates_);
    lfProfilesTitle_ = new QLabel("Profile selection:", page);
    col->addWidget(lfProfilesTitle_);
    lfProfiles_ = new QWidget(page);
    new QFormLayout(lfProfiles_);
    lfProfiles_->layout()->setContentsMargins(0, 0, 0, 0);
    col->addWidget(lfProfiles_);
    col->addWidget(separator(page));
    lfSolid_ = new QCheckBox("Solid", page);
    lfSolid_->setFocusPolicy(Qt::NoFocus);
    col->addWidget(lfSolid_);
    col->addWidget(separator(page));
    lfOk_ = button("OK [Enter]", page);
    auto* cancel = button("Cancel [Esc]", page);
    col->addLayout(okCancel(lfOk_, cancel));

    connect(lfSolid_, &QCheckBox::clicked, this, [a](bool on) { a->post([a, on] { a->setLoftSolid(on); }); });
    connect(lfOk_, &QPushButton::clicked, this, [a] { a->post([a] { a->commitLoft(); }); });
    connect(cancel, &QPushButton::clicked, this, [a] { a->post([a] { a->cancelLoft(); }); });
    return page;
}

QWidget* ToolPanel::buildBoolean() {
    auto* page = new QWidget(this);
    auto* col = new QVBoxLayout(page);
    col->setContentsMargins(0, 0, 0, 0);
    App* a = &app_;

    blHint_ = new QLabel(page);
    blHint_->setWordWrap(true);
    col->addWidget(blHint_);
    col->addWidget(separator(page));
    auto* targetRow = new QHBoxLayout;
    blTarget_ = new QLabel(page);
    blClearTarget_ = button("Clear", page);
    targetRow->addWidget(blTarget_, 1);
    targetRow->addWidget(blClearTarget_);
    col->addLayout(targetRow);
    auto* toolRow = new QHBoxLayout;
    blTool_ = new QLabel(page);
    blClearTool_ = button("Clear", page);
    toolRow->addWidget(blTool_, 1);
    toolRow->addWidget(blClearTool_);
    col->addLayout(toolRow);
    blToolHint_ = new QLabel(page);
    blToolHint_->setStyleSheet("color: #b08000;");
    col->addWidget(blToolHint_);
    col->addWidget(separator(page));
    blApply_ = button("Apply [Enter]", page);
    col->addWidget(blApply_);
    auto* cancel = button("Cancel [Esc]", page);
    col->addWidget(cancel);
    blPreview_ = new QLabel("Preview ready", page);
    blPreview_->setStyleSheet("color: #2e8b30;");
    col->addWidget(blPreview_);

    connect(blClearTarget_, &QPushButton::clicked, this, [a] { a->post([a] { a->clearBooleanTarget(); }); });
    connect(blClearTool_, &QPushButton::clicked, this, [a] { a->post([a] { a->clearBooleanTool(); }); });
    connect(blApply_, &QPushButton::clicked, this, [a] { a->post([a] { a->commitBoolean(); }); });
    connect(cancel, &QPushButton::clicked, this, [a] { a->post([a] { a->cancelBoolean(); }); });
    return page;
}

// ---- refresh -------------------------------------------------------------

void ToolPanel::refreshExtrude(const App::ExtrudePanelModel& m) {
    setLabel(title_, "Extrude");
    if (auto* items = qobject_cast<QStandardItemModel*>(exOp_->model()))
        items->item(1)->setEnabled(m.cutAllowed || m.operation == 1);
    setCombo(exOp_, m.operation);
    setField(exDist_, m.distanceText);
    setCombo(exDir_, m.direction);
    setField(exOffset_, m.offsetText);
    setLabel(exProfiles_, QString("Profiles: %1 / %2").arg(m.selectedProfiles).arg(m.totalProfiles));
    exOk_->setEnabled(m.canCommit);
}

void ToolPanel::refreshRevolve(const App::RevolvePanelModel& m) {
    setLabel(title_, "Revolve");
    setCombo(rvOp_, m.operation);
    setField(rvAngle_, m.angleText);
    const bool hasAxis = m.axisLine != 0;
    rvAxis_->setVisible(hasAxis);
    rvChangeAxis_->setVisible(hasAxis);
    if (hasAxis) setLabel(rvAxis_, QString("Axis: Line %1").arg(m.axisLine));
    rvPickHint_->setVisible(!hasAxis && m.selectingAxis);
    rvSelectAxis_->setVisible(!hasAxis && !m.selectingAxis);
    setLabel(rvProfiles_, QString("Profiles: %1 / %2").arg(m.selectedProfiles).arg(m.totalProfiles));
    rvSelectProfiles_->setVisible(!m.selectingAxis);
    rvOk_->setEnabled(m.canCommit);
}

void ToolPanel::refreshLoft(const App::LoftPanelModel& m) {
    setLabel(title_, "Loft");
    App* a = &app_;

    std::vector<int> sections;
    for (const auto& s : m.sections) sections.insert(sections.end(), {s.plane, s.profile, s.profileCount});
    if (sections != shownSections_) {
        shownSections_ = sections;
        clearLayout(lfSections_->layout());
        for (int i = 0; i < (int)m.sections.size(); i++) {
            auto* row = new QWidget(lfSections_);
            auto* h = new QHBoxLayout(row);
            h->setContentsMargins(0, 0, 0, 0);
            h->addWidget(new QLabel(QString("Section %1: Plane %2, Profile %3")
                .arg(i + 1).arg(m.sections[i].plane).arg(m.sections[i].profile), row), 1);
            auto* remove = button("X", row);
            remove->setFixedWidth(24);
            connect(remove, &QPushButton::clicked, this, [a, i] { a->post([a, i] { a->removeLoftSection(i); }); });
            h->addWidget(remove);
            lfSections_->layout()->addWidget(row);
        }
    }

    std::vector<std::string> candidates;
    for (const auto& c : m.candidates) candidates.push_back(std::to_string(c.plane) + ":" + c.label);
    if (candidates != shownCandidates_) {
        shownCandidates_ = candidates;
        clearLayout(lfCandidates_->layout());
        for (const auto& c : m.candidates) {
            auto* add = button(QString::fromStdString(c.label), lfCandidates_);
            const int plane = c.plane;
            connect(add, &QPushButton::clicked, this, [a, plane] { a->post([a, plane] { a->addLoftSection(plane); }); });
            lfCandidates_->layout()->addWidget(add);
        }
    }

    // A profile choice for each section with more than one profile
    std::vector<int> counts;
    for (const auto& s : m.sections) counts.push_back(s.profileCount);
    auto* form = static_cast<QFormLayout*>(lfProfiles_->layout());
    if (counts != shownProfileCounts_) {
        shownProfileCounts_ = counts;
        while (form->rowCount() > 0) form->removeRow(0);
        for (int i = 0; i < (int)m.sections.size(); i++) {
            if (m.sections[i].profileCount <= 1) continue;
            auto* spin = new QSpinBox(lfProfiles_);
            spin->setRange(0, m.sections[i].profileCount - 1);
            spin->setKeyboardTracking(false);
            spin->setProperty("section", i);
            connect(spin, &QSpinBox::valueChanged, this, [a, i](int v) {
                a->post([a, i, v] { a->setLoftSectionProfile(i, v); });
            });
            form->addRow(QString("Section %1 profile").arg(i + 1), spin);
        }
    }
    bool anyChoice = false;
    for (int r = 0; r < form->rowCount(); r++) {
        QLayoutItem* item = form->itemAt(r, QFormLayout::FieldRole);
        auto* spin = item ? qobject_cast<QSpinBox*>(item->widget()) : nullptr;
        if (!spin) continue;
        anyChoice = true;
        const int section = spin->property("section").toInt();
        if (!spin->hasFocus() && section < (int)m.sections.size() && spin->value() != m.sections[section].profile) {
            const QSignalBlocker block(spin);
            spin->setValue(m.sections[section].profile);
        }
    }
    lfProfilesTitle_->setVisible(anyChoice);
    lfProfiles_->setVisible(anyChoice);

    if (lfSolid_->isChecked() != m.solid) {
        const QSignalBlocker block(lfSolid_);
        lfSolid_->setChecked(m.solid);
    }
    lfOk_->setEnabled(m.canCommit);
}

void ToolPanel::refreshBoolean(const App::BooleanPanelModel& m) {
    setLabel(title_, m.isUnion ? "Union Bodies" : "Subtract Bodies");
    setLabel(blHint_, m.isUnion ? "Pick two bodies to combine into one."
                                : "Pick target body, then tool body to cut away.");
    const bool hasTarget = m.target >= 0, hasTool = m.tool >= 0;
    setLabel(blTarget_, hasTarget ? QString("Target: Body %1").arg(m.target)
                                  : QString("Click to select target body"));
    blTarget_->setStyleSheet(hasTarget ? "" : "color: #b08000;");
    blClearTarget_->setVisible(hasTarget);
    blTool_->setVisible(hasTool);
    blClearTool_->setVisible(hasTool);
    if (hasTool) setLabel(blTool_, QString(m.isUnion ? "Other: Body %1" : "Tool: Body %1").arg(m.tool));
    blToolHint_->setVisible(!hasTool && hasTarget);
    if (!hasTool && hasTarget)
        setLabel(blToolHint_, m.isUnion ? "Click another body to combine" : "Click body to subtract");
    blApply_->setEnabled(m.canCommit);
    blPreview_->setVisible(m.previewValid);
}

void ToolPanel::refresh() {
    Page page = None;
    const App::ExtrudePanelModel ex = app_.extrudePanelModel();
    const App::RevolvePanelModel rv = ex.open ? App::RevolvePanelModel{} : app_.revolvePanelModel();
    const App::BooleanPanelModel bl = app_.booleanPanelModel();
    App::LoftPanelModel lf;
    if (ex.open) page = Extrude;
    else if (rv.open) page = Revolve;
    else if (bl.open) page = Boolean;
    else if ((lf = app_.loftPanelModel()).open) page = Loft;

    if (page == None) {
        if (!isHidden()) hide();
        shown_ = None;
        return;
    }
    if (page != shown_) {
        shown_ = page;
        pages_->setCurrentIndex(page);
        setFixedWidth(page == Loft ? 260 : 240);
        // Only the current page counts towards the height.
        for (int i = 0; i < pages_->count(); i++)
            pages_->widget(i)->setSizePolicy(QSizePolicy::Preferred,
                i == page ? QSizePolicy::Preferred : QSizePolicy::Ignored);
    }
    switch (page) {
    case Extrude: refreshExtrude(ex); break;
    case Revolve: refreshRevolve(rv); break;
    case Loft: refreshLoft(lf); break;
    case Boolean: refreshBoolean(bl); break;
    case None: break;
    }

    const int h = sizeHint().height();
    if (height() != h) resize(width(), h);
    const QPoint at(viewport_->width() - width() - 8, 8);
    if (pos() != at) move(at);
    if (isHidden()) { show(); raise(); }
}

void ToolPanel::keyPressEvent(QKeyEvent* e) {
    // Escape in a field leaves the field without applying it.
    if (e->key() == Qt::Key_Escape) {
        if (auto* field = qobject_cast<QLineEdit*>(focusWidget()); field && field->hasFocus()) {
            const QSignalBlocker block(field);
            viewport_->setFocus();
        }
    }
    e->accept();
}

void ToolPanel::keyReleaseEvent(QKeyEvent* e) { e->accept(); }

} // namespace shitcad
