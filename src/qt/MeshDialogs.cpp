#include "MeshDialogs.h"
#include "UnitUtils.h"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include <algorithm>

namespace shitcad {

namespace {

QComboBox* unitCombo(QWidget* parent) {
    auto* c = new QComboBox(parent);
    for (int i = 0; i < kUnitCount; i++) c->addItem(QString::fromUtf8(kUnits[i].name));
    return c;
}

QPushButton* pushButton(QWidget* parent, const QString& text, std::function<void()> onClick) {
    auto* b = new QPushButton(text, parent);
    QObject::connect(b, &QPushButton::clicked, parent, [onClick] { onClick(); });
    return b;
}

} // namespace

// ---- import ------------------------------------------------------------------

MeshImportDialog::MeshImportDialog(App& app, QWidget* parent) : QDialog(parent), app_(app) {
    setWindowTitle("Import Mesh");
    setModal(false);

    auto* root = new QVBoxLayout(this);
    path_ = new QLabel(this);
    path_->setWordWrap(true);
    root->addWidget(path_);

    error_ = new QLabel(this);
    error_->setWordWrap(true);
    error_->setStyleSheet("color: #d05050;");
    root->addWidget(error_);

    form_ = new QWidget(this);
    auto* form = new QFormLayout(form_);
    triangles_ = new QLabel(form_);
    form->addRow(triangles_);
    name_ = new QLineEdit(form_);
    form->addRow("Name", name_);
    unit_ = unitCombo(form_);
    form->addRow("Unit of the numbers in this file", unit_);
    size_ = new QLabel(form_);
    form->addRow(size_);
    warning_ = new QLabel(form_);
    warning_->setWordWrap(true);
    warning_->setStyleSheet("color: #b08000;");
    form->addRow(warning_);
    root->addWidget(form_);

    App* a = &app_;
    auto* buttons = new QHBoxLayout;
    buttons->addStretch(1);
    buttons->addWidget(pushButton(this, "Import", [a] { a->post([a] { a->confirmMeshImport(); }); }));
    buttons->addWidget(pushButton(this, "Cancel", [a] { a->post([a] { a->cancelMeshImport(); }); }));
    root->addLayout(buttons);

    connect(name_, &QLineEdit::textEdited, this, [a](const QString& t) {
        const std::string s = t.toUtf8().toStdString();
        a->post([a, s] { a->setMeshImportName(s); });
    });
    connect(unit_, &QComboBox::currentIndexChanged, this, [a](int i) {
        a->post([a, i] { a->setMeshImportUnit(i); });
    });
    resize(420, 100);
}

void MeshImportDialog::refresh() {
    const App::MeshImportModel m = app_.meshImportModel();
    if (m.open != isVisible()) setVisible(m.open);
    if (!m.open) {
        shownPath_.clear();
        return;
    }

    path_->setText(QString::fromStdString(m.path));
    const bool failed = !m.error.empty();
    error_->setVisible(failed);
    error_->setText(QString::fromStdString(m.error));
    form_->setVisible(!failed);
    if (failed) return;

    if (m.path != shownPath_) { // a new file: take App's defaults
        shownPath_ = m.path;
        const QSignalBlocker b1(name_), b2(unit_);
        name_->setText(QString::fromStdString(m.name));
        unit_->setCurrentIndex(m.unitIndex);
    }
    triangles_->setText(QString("%1 triangles").arg(m.triangles));
    if (m.maxExtMm >= 1000.0f)
        size_->setText(QString("Size: %1 x %2 x %3 m").arg(m.extMm[0] / 1000.0, 0, 'g', 4)
                           .arg(m.extMm[1] / 1000.0, 0, 'g', 4).arg(m.extMm[2] / 1000.0, 0, 'g', 4));
    else
        size_->setText(QString("Size: %1 x %2 x %3 mm").arg(m.extMm[0], 0, 'g', 4)
                           .arg(m.extMm[1], 0, 'g', 4).arg(m.extMm[2], 0, 'g', 4));
    warning_->setVisible(m.sizeSuspicious);
    if (m.sizeSuspicious)
        warning_->setText(QString("That is %1 mm across. If the real part is not that size, "
                                  "the unit above is wrong.").arg(m.maxExtMm, 0, 'g', 4));
    adjustSize();
}

void MeshImportDialog::reject() {
    App* a = &app_;
    app_.post([a] { a->cancelMeshImport(); });
    QDialog::reject();
}

// ---- placement ---------------------------------------------------------------

MeshPlacePanel::MeshPlacePanel(App& app, QWidget* parent) : QDialog(parent), app_(app) {
    setWindowTitle("Place");
    setModal(false);
    setWindowFlag(Qt::Tool); // stays above the window without taking it over

    App* a = &app_;
    auto* root = new QVBoxLayout(this);
    error_ = new QLabel(this);
    error_->setWordWrap(true);
    error_->setStyleSheet("color: #d05050;");
    root->addWidget(error_);

    form_ = new QWidget(this);
    auto* col = new QVBoxLayout(form_);
    {
        auto* row = new QHBoxLayout;
        row->addWidget(new QLabel("File unit", form_));
        unit_ = unitCombo(form_);
        row->addWidget(unit_, 1);
        col->addLayout(row);
    }
    size_ = new QLabel(form_);
    bottom_ = new QLabel(form_);
    size_->setEnabled(false);
    bottom_->setEnabled(false);
    col->addWidget(size_);
    col->addWidget(bottom_);

    col->addWidget(new QLabel("Rotate 90 deg about its centre", form_));
    {
        auto* grid = new QGridLayout;
        const char* axisNames[3] = {"X", "Y", "Z"};
        for (int axis = 0; axis < 3; axis++) {
            grid->addWidget(new QLabel(axisNames[axis], form_), axis, 0);
            int c = 1;
            for (double deg : {-90.0, 90.0, 180.0}) {
                grid->addWidget(pushButton(form_, deg == 180.0 ? "180" : (deg < 0 ? "-90" : "+90"),
                                           [a, axis, deg] { a->post([a, axis, deg] { a->meshPlaceRotate(axis, deg); }); }),
                                axis, c++);
            }
        }
        col->addLayout(grid);
    }
    // This viewport is Y-up. CAD packages including Onshape export Z-up, which
    // lands on its side here; Rx(-90) takes +Z to +Y.
    col->addWidget(pushButton(form_, "Z-up file (Onshape) -> stand upright",
                              [a] { a->post([a] { a->meshPlaceRotate(0, -90.0); }); }));
    {
        auto* row = new QHBoxLayout;
        angle_ = new QDoubleSpinBox(form_);
        angle_->setRange(-360.0, 360.0);
        angle_->setDecimals(2);
        angle_->setKeyboardTracking(false);
        row->addWidget(angle_);
        row->addWidget(new QLabel("deg", form_));
        angleAxis_ = new QComboBox(form_);
        angleAxis_->addItems({"X", "Y", "Z"});
        row->addWidget(angleAxis_);
        row->addWidget(pushButton(form_, "Rotate", [this, a] {
            const double deg = angle_->value();
            const int axis = angleAxis_->currentIndex();
            a->post([a, axis, deg] { a->meshPlaceRotate(axis, deg); });
        }), 1);
        col->addLayout(row);
    }

    col->addWidget(new QLabel("Move (mm, applied after rotation)", form_));
    const char* posNames[3] = {"X", "Y", "Z"};
    for (int i = 0; i < 3; i++) {
        pos_[i] = new QDoubleSpinBox(form_);
        pos_[i]->setRange(-1e9, 1e9);
        pos_[i]->setDecimals(3);
        pos_[i]->setPrefix(QString("%1 ").arg(posNames[i]));
        pos_[i]->setKeyboardTracking(false); // each apply replays the whole model
        col->addWidget(pos_[i]);
        connect(pos_[i], &QDoubleSpinBox::editingFinished, this, [this] { pushPosition(); });
    }
    col->addWidget(pushButton(form_, "Drop to ground (Y = 0)", [a] { a->post([a] { a->meshPlaceDropToGround(); }); }));
    col->addWidget(pushButton(form_, "Centre on origin (X, Z)", [a] { a->post([a] { a->meshPlaceCentreOnOrigin(); }); }));
    col->addWidget(pushButton(form_, "Reset placement", [a] { a->post([a] { a->meshPlaceResetPlacement(); }); }));
    root->addWidget(form_);

    auto* buttons = new QHBoxLayout;
    buttons->addStretch(1);
    buttons->addWidget(pushButton(this, "Done", [this, a] {
        closing_ = true;
        a->post([a] { a->finishMeshPlace(true); });
    }));
    buttons->addWidget(pushButton(this, "Cancel", [this, a] {
        closing_ = true;
        a->post([a] { a->finishMeshPlace(false); });
    }));
    root->addLayout(buttons);

    connect(unit_, &QComboBox::currentIndexChanged, this, [a](int i) {
        a->post([a, i] { a->meshPlaceSetUnit(i); });
    });
    connect(angle_, &QDoubleSpinBox::valueChanged, this, [this, a] {
        const double deg = angle_->value();
        const int axis = angleAxis_->currentIndex();
        a->post([a, deg, axis] { a->meshPlaceSetAngle((float)deg, axis); });
    });
}

void MeshPlacePanel::pushPosition() {
    App* a = &app_;
    double p[3] = {pos_[0]->value(), pos_[1]->value(), pos_[2]->value()};
    a->post([a, p] { a->meshPlaceSetPosition(p); });
}

void MeshPlacePanel::refresh() {
    const App::MeshPlaceModel m = app_.meshPlaceModel();
    if (m.active != isVisible()) {
        setVisible(m.active);
        closing_ = false;
    }
    if (!m.active) return;

    setWindowTitle(QString("Place: %1").arg(QString::fromStdString(m.name)));
    const bool failed = !m.error.empty();
    error_->setVisible(failed);
    error_->setText(QString::fromStdString(m.error));
    form_->setVisible(!failed);
    if (failed) return;

    const double big = std::max({m.hi[0] - m.lo[0], m.hi[1] - m.lo[1], m.hi[2] - m.lo[2]});
    const double s = big >= 1000.0 ? 0.001 : 1.0;
    const char* su = big >= 1000.0 ? "m" : "mm";
    size_->setText(QString("Size  X %1  Y %2  Z %3 %4")
                       .arg((m.hi[0] - m.lo[0]) * s, 0, 'g', 4).arg((m.hi[1] - m.lo[1]) * s, 0, 'g', 4)
                       .arg((m.hi[2] - m.lo[2]) * s, 0, 'g', 4).arg(su));
    // The viewport is Y-up (ground grid in XZ), so "bottom" is along Y.
    bottom_->setText(QString("Bottom at Y = %1 %2").arg(m.lo[1] * s, 0, 'g', 4).arg(su));

    const QSignalBlocker b1(unit_), b2(angle_), b3(angleAxis_);
    unit_->setCurrentIndex(m.unitIndex);
    if (!angle_->hasFocus()) angle_->setValue(m.angleDeg);
    angleAxis_->setCurrentIndex(m.angleAxis);
    for (int i = 0; i < 3; i++) {
        if (pos_[i]->hasFocus()) continue; // never overwrite what is being typed
        const QSignalBlocker b(pos_[i]);
        pos_[i]->setValue(m.pos[i]);
    }
    adjustSize();
}

void MeshPlacePanel::reject() {
    if (!closing_) { // the window's X or Escape: like Cancel
        App* a = &app_;
        app_.post([a] { a->finishMeshPlace(false); });
    }
    QDialog::reject();
}

} // namespace shitcad
