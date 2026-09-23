#include "PlaneDialogs.h"

#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QSignalBlocker>
#include <QSlider>
#include <QVBoxLayout>

#include <cmath>

namespace shitcad {

// ---- Add Reference Plane -------------------------------------------------------

AddPlaneDialog::AddPlaneDialog(App& app, QWidget* parent) : QDialog(parent), app_(app) {
    setWindowTitle("Add Reference Plane");
    setModal(false);
    App* a = &app_;

    auto* root = new QVBoxLayout(this);
    root->addWidget(new QLabel("Source:", this));

    fromPlane_ = new QRadioButton("From Plane", this);
    root->addWidget(fromPlane_);
    sourceList_ = new QWidget(this);
    sourceLayout_ = new QVBoxLayout(sourceList_);
    sourceLayout_->setContentsMargins(20, 0, 0, 0);
    root->addWidget(sourceList_);

    fromFace_ = new QRadioButton("From Face", this);
    root->addWidget(fromFace_);
    faceState_ = new QLabel(this);
    faceState_->setContentsMargins(20, 0, 0, 0);
    root->addWidget(faceState_);

    root->addWidget(new QLabel("Offset distance:", this));
    offset_ = new QLineEdit(this);
    root->addWidget(offset_);
    root->addWidget(new QLabel("Name (optional):", this));
    name_ = new QLineEdit(this);
    root->addWidget(name_);

    auto* buttons = new QHBoxLayout;
    create_ = new QPushButton("Create", this);
    auto* cancel = new QPushButton("Cancel", this);
    buttons->addWidget(create_);
    buttons->addWidget(cancel);
    buttons->addStretch(1);
    root->addLayout(buttons);

    connect(fromPlane_, &QRadioButton::clicked, this, [a] { a->post([a] { a->setAddPlaneSource(false, -1); }); });
    connect(fromFace_, &QRadioButton::clicked, this, [a] { a->post([a] { a->setAddPlaneSource(true, -1); }); });
    connect(offset_, &QLineEdit::textEdited, this, [a](const QString& t) {
        const std::string s = t.toUtf8().toStdString();
        a->post([a, s] { a->setAddPlaneOffsetText(s); });
    });
    connect(name_, &QLineEdit::textEdited, this, [a](const QString& t) {
        const std::string s = t.toUtf8().toStdString();
        a->post([a, s] { a->setAddPlaneName(s); });
    });
    connect(create_, &QPushButton::clicked, this, [a] { a->post([a] { a->createOffsetPlane(); }); });
    connect(cancel, &QPushButton::clicked, this, [a] { a->post([a] { a->cancelAddPlane(); }); });
}

void AddPlaneDialog::rebuildSources(const App::AddPlaneModel& m) {
    for (QRadioButton* b : sources_) { sourceLayout_->removeWidget(b); b->deleteLater(); }
    sources_.clear();
    App* a = &app_;
    for (const auto& s : m.sources) {
        auto* b = new QRadioButton(QString::fromStdString(s.name), sourceList_);
        const int index = s.index;
        connect(b, &QRadioButton::clicked, this, [a, index] {
            a->post([a, index] { a->setAddPlaneSource(false, index); });
        });
        sourceLayout_->addWidget(b);
        sources_.push_back(b);
    }
    shownSources_ = m.sources.size();
}

void AddPlaneDialog::refresh() {
    const App::AddPlaneModel m = app_.addPlaneModel();
    if (m.open != isVisible()) setVisible(m.open);
    if (!m.open) return;

    if (m.sources.size() != shownSources_) rebuildSources(m);

    const QSignalBlocker b1(fromPlane_), b2(fromFace_);
    fromPlane_->setChecked(!m.fromFace);
    fromFace_->setChecked(m.fromFace);
    sourceList_->setVisible(!m.fromFace);
    for (size_t i = 0; i < sources_.size() && i < m.sources.size(); i++) {
        const QSignalBlocker b(sources_[i]);
        sources_[i]->setChecked(m.sources[i].index == m.sourceIndex);
    }
    faceState_->setVisible(m.fromFace);
    if (m.fromFace) {
        faceState_->setText(m.waitingFace ? "Click a face in the viewport" : "Face selected");
        faceState_->setStyleSheet(m.waitingFace ? "color: #b08000;" : "color: #2e8b30;");
    }
    if (!offset_->hasFocus()) {
        const QSignalBlocker b(offset_);
        offset_->setText(QString::fromStdString(m.offsetText));
    }
    if (!name_->hasFocus()) {
        const QSignalBlocker b(name_);
        name_->setText(QString::fromStdString(m.name));
    }
    create_->setEnabled(m.canCreate);
    adjustSize();
}

void AddPlaneDialog::reject() {
    App* a = &app_;
    app_.post([a] { a->cancelAddPlane(); });
    QDialog::reject();
}

// ---- Tangent plane -------------------------------------------------------------

TangentPlaneDialog::TangentPlaneDialog(App& app, QWidget* parent) : QDialog(parent), app_(app) {
    setWindowTitle("Tangent Plane");
    setModal(false);
    App* a = &app_;

    auto* root = new QVBoxLayout(this);
    root->addWidget(new QLabel("Create a tangent plane on cylinder", this));
    root->addWidget(new QLabel("Name:", this));
    name_ = new QLineEdit(this);
    root->addWidget(name_);

    root->addWidget(new QLabel("Angle (degrees from click point):", this));
    angle_ = new QDoubleSpinBox(this);
    angle_->setRange(-180.0, 180.0);
    angle_->setDecimals(1);
    angle_->setSuffix(" deg");
    angle_->setKeyboardTracking(false);
    root->addWidget(angle_);
    slider_ = new QSlider(Qt::Horizontal, this);
    slider_->setRange(-1800, 1800);   // tenths of a degree
    root->addWidget(slider_);

    auto* create = new QPushButton("Create && Sketch", this);
    auto* cancel = new QPushButton("Cancel", this);
    root->addWidget(create);
    root->addWidget(cancel);

    connect(name_, &QLineEdit::textEdited, this, [a](const QString& t) {
        const std::string s = t.toUtf8().toStdString();
        a->post([a, s] { a->setTangentPlaneName(s); });
    });
    auto setAngle = [a](double deg) { a->post([a, deg] { a->setTangentPlaneAngle((float)deg); }); };
    connect(angle_, &QDoubleSpinBox::valueChanged, this, [setAngle](double v) { setAngle(v); });
    connect(slider_, &QSlider::valueChanged, this, [setAngle](int v) { setAngle(v / 10.0); });
    connect(create, &QPushButton::clicked, this, [a] { a->post([a] { a->createTangentPlane(); }); });
    connect(cancel, &QPushButton::clicked, this, [a] { a->post([a] { a->cancelTangentPlane(); }); });
}

void TangentPlaneDialog::refresh() {
    const App::TangentPlaneModel m = app_.tangentPlaneModel();
    if (m.open != isVisible()) setVisible(m.open);
    if (!m.open) return;

    if (!name_->hasFocus()) {
        const QSignalBlocker b(name_);
        name_->setText(QString::fromStdString(m.name));
    }
    if (!angle_->hasFocus()) {
        const QSignalBlocker b(angle_);
        angle_->setValue(m.angleDeg);
    }
    if (!slider_->isSliderDown()) {
        const QSignalBlocker b(slider_);
        slider_->setValue((int)std::lround(m.angleDeg * 10.0f));
    }
}

void TangentPlaneDialog::reject() {
    App* a = &app_;
    app_.post([a] { a->cancelTangentPlane(); });
    QDialog::reject();
}

} // namespace shitcad
