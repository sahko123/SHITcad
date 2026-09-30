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
    blocked_ = new QLabel(this);
    blocked_->setStyleSheet("color: #b08000;");
    root->addWidget(blocked_);

    App* a = &app_;
    auto* buttons = new QHBoxLayout;
    buttons->addStretch(1);
    import_ = pushButton(this, "Import", [a] { a->post([a] { a->confirmMeshImport(); }); });
    buttons->addWidget(import_);
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
    blocked_->setVisible(!failed && !m.blocked.empty());
    blocked_->setText(QString::fromStdString(m.blocked));
    import_->setEnabled(!failed && m.blocked.empty());
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

// ---- STEP / IGES import ------------------------------------------------------

namespace {

QString sizeText(const double ext[3]) {
    const double big = std::max({ext[0], ext[1], ext[2]});
    const double s = big >= 1000.0 ? 0.001 : 1.0;
    return QString("Size: %1 x %2 x %3 %4").arg(ext[0] * s, 0, 'g', 4).arg(ext[1] * s, 0, 'g', 4)
                                           .arg(ext[2] * s, 0, 'g', 4).arg(big >= 1000.0 ? "m" : "mm");
}

} // namespace

CadImportDialog::CadImportDialog(App& app, QWidget* parent) : QDialog(parent), app_(app) {
    setWindowTitle("Import");
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
    contents_ = new QLabel(form_);
    form->addRow(contents_);
    size_ = new QLabel(form_);
    form->addRow(size_);
    warning_ = new QLabel(form_);
    warning_->setWordWrap(true);
    warning_->setStyleSheet("color: #b08000;");
    form->addRow(warning_);
    name_ = new QLineEdit(form_);
    form->addRow("Name", name_);
    up_ = new QComboBox(form_);
    // This viewport is Y-up. Onshape, Fusion and most CAD export Z-up.
    up_->addItems({"Z (most CAD: Onshape, Fusion, Inventor)", "Y (SolidWorks, as-is)"});
    form->addRow("Up axis in the file", up_);
    root->addWidget(form_);
    blocked_ = new QLabel(this);
    blocked_->setStyleSheet("color: #b08000;");
    root->addWidget(blocked_);

    App* a = &app_;
    auto* buttons = new QHBoxLayout;
    buttons->addStretch(1);
    import_ = pushButton(this, "Import", [a] { a->post([a] { a->confirmCadImport(); }); });
    buttons->addWidget(import_);
    buttons->addWidget(pushButton(this, "Cancel", [a] { a->post([a] { a->cancelCadImport(); }); }));
    root->addLayout(buttons);

    connect(name_, &QLineEdit::textEdited, this, [a](const QString& t) {
        const std::string s = t.toUtf8().toStdString();
        a->post([a, s] { a->setCadImportName(s); });
    });
    connect(up_, &QComboBox::currentIndexChanged, this, [a](int i) {
        a->post([a, i] { a->setCadImportZUp(i == 0); });
    });
    resize(460, 100);
}

void CadImportDialog::refresh() {
    const App::CadImportModel m = app_.cadImportModel();
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
    blocked_->setVisible(!failed && !m.blocked.empty());
    blocked_->setText(QString::fromStdString(m.blocked));
    import_->setEnabled(!failed && m.blocked.empty());
    if (failed) {
        adjustSize();
        return;
    }

    if (m.path != shownPath_) { // a new file: take App's defaults
        shownPath_ = m.path;
        const QSignalBlocker b1(name_), b2(up_);
        name_->setText(QString::fromStdString(m.name));
        up_->setCurrentIndex(m.zUp ? 0 : 1);
    }
    setWindowTitle(QString("Import %1").arg(QString::fromStdString(m.format)));
    QString what;
    if (m.solids) what += QString("%1 solid%2").arg(m.solids).arg(m.solids == 1 ? "" : "s");
    if (m.surfaces) {
        if (!what.isEmpty()) what += ", ";
        what += QString("%1 surface body%2").arg(m.surfaces).arg(m.surfaces == 1 ? "" : "s");
    }
    if (!m.fileUnit.empty()) what += QString("  (file unit: %1, converted to mm)").arg(QString::fromStdString(m.fileUnit));
    contents_->setText(what);
    size_->setText(sizeText(m.extMm));
    warning_->setVisible(m.skippedWires > 0);
    if (m.skippedWires > 0)
        warning_->setText("The file also has curves or points; they make no body and are left out.");
    adjustSize();
}

void CadImportDialog::reject() {
    App* a = &app_;
    app_.post([a] { a->cancelCadImport(); });
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
        unitRow_ = new QWidget(form_);
        auto* row = new QHBoxLayout(unitRow_);
        row->setContentsMargins(0, 0, 0, 0);
        row->addWidget(new QLabel("File unit", unitRow_));
        unit_ = unitCombo(unitRow_);
        row->addWidget(unit_, 1);
        col->addWidget(unitRow_);
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
    col->addWidget(pushButton(form_, "Z-up file -> stand upright",
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
    unitRow_->setVisible(m.hasUnit);
    if (m.hasUnit) unit_->setCurrentIndex(m.unitIndex);
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
