#include "SectionControls.h"

#include <QButtonGroup>
#include <QCheckBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QRadioButton>
#include <QSignalBlocker>
#include <QSlider>
#include <QVBoxLayout>

#include <cmath>

namespace shitcad {

namespace {
constexpr int kSteps = 1000;   // slider resolution across the scene's extent

QLabel* note(const QString& text, QWidget* parent) {
    auto* label = new QLabel(text, parent);
    label->setWordWrap(true);
    label->setEnabled(false);   // greyed
    return label;
}

void setChecked(QAbstractButton* b, bool on) {
    if (b->isChecked() == on) return;
    const QSignalBlocker block(b);
    b->setChecked(on);
}
}

SectionControls::SectionControls(App& app, QWidget* parent) : QWidget(parent), app_(app) {
    App* a = &app_;
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(0, 0, 0, 0);

    enabled_ = new QCheckBox("Section view", this);
    root->addWidget(enabled_);
    offHint_ = note("Cut the model open to see inside.", this);
    root->addWidget(offHint_);

    body_ = new QWidget(this);
    auto* col = new QVBoxLayout(body_);
    col->setContentsMargins(0, 0, 0, 0);
    auto* axisRow = new QHBoxLayout;
    axisRow->addWidget(new QLabel("Cut along", body_));
    auto* group = new QButtonGroup(this);
    const char* names[] = {"X", "Y", "Z"};
    for (int i = 0; i < 3; i++) {
        axis_[i] = new QRadioButton(names[i], body_);
        group->addButton(axis_[i], i);
        axisRow->addWidget(axis_[i]);
    }
    flip_ = new QCheckBox("Flip", body_);
    axisRow->addWidget(flip_);
    axisRow->addStretch(1);
    col->addLayout(axisRow);

    auto* posRow = new QHBoxLayout;
    slider_ = new QSlider(Qt::Horizontal, body_);
    slider_->setRange(0, kSteps);
    posRow->addWidget(slider_, 1);
    position_ = new QLabel(body_);
    position_->setMinimumWidth(60);
    position_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    posRow->addWidget(position_);
    col->addLayout(posRow);

    auto* capRow = new QHBoxLayout;
    auto* centre = new QPushButton("Centre", body_);
    capRow->addWidget(centre);
    cap_ = new QCheckBox("Cap the cut", body_);
    capRow->addWidget(cap_);
    capRow->addStretch(1);
    col->addLayout(capRow);
    col->addWidget(note("Cuts geometry and results. Spray cones stay whole.", body_));
    openNote_ = note("", body_);
    col->addWidget(openNote_);
    watertightNote_ = note("(A cap needs a watertight surface.)", body_);
    col->addWidget(watertightNote_);
    root->addWidget(body_);

    connect(enabled_, &QCheckBox::clicked, this, [a](bool on) { a->post([a, on] { a->setSectionEnabled(on); }); });
    connect(group, &QButtonGroup::idClicked, this, [a](int axis) { a->post([a, axis] { a->setSectionAxis(axis); }); });
    connect(flip_, &QCheckBox::clicked, this, [a](bool on) { a->post([a, on] { a->setSectionFlip(on); }); });
    connect(slider_, &QSlider::valueChanged, this, [this, a](int v) {
        const float mm = sliderMm(v);
        position_->setText(QString::asprintf("%.0f mm", mm));
        a->post([a, mm] { a->setSectionPosition(mm); });
    });
    connect(centre, &QPushButton::clicked, this, [a] { a->post([a] { a->centreSection(); }); });
    connect(cap_, &QCheckBox::clicked, this, [a](bool on) { a->post([a, on] { a->setSectionCap(on); }); });
}

float SectionControls::sliderMm(int value) const {
    return lo_ + (hi_ - lo_) * (float)value / (float)kSteps;
}

void SectionControls::refresh(const App::SectionModel& m) {
    setChecked(enabled_, m.enabled);
    offHint_->setVisible(!m.enabled);
    body_->setVisible(m.enabled);
    if (!m.enabled) return;

    // Shown means clamped: while the controls are up, the plane is pulled
    // back into the scene if the scene shrank.
    if (m.outOfRange) {
        App* a = &app_;
        const float mm = m.position;
        app_.post([a, mm] { a->setSectionPosition(mm); });
    }

    for (int i = 0; i < 3; i++) setChecked(axis_[i], m.axis == i);
    setChecked(flip_, m.flip);
    lo_ = m.lo;
    hi_ = m.hi;
    if (!slider_->isSliderDown()) {
        const int v = hi_ > lo_ ? (int)std::lround((m.position - lo_) / (hi_ - lo_) * kSteps) : 0;
        if (slider_->value() != v) {
            const QSignalBlocker block(slider_);
            slider_->setValue(v);
        }
        const QString text = QString::asprintf("%.0f mm", m.position);
        if (position_->text() != text) position_->setText(text);
    }
    setChecked(cap_, m.cap);
    const bool openShown = m.cap && m.openSurfaces > 0;
    openNote_->setVisible(openShown);
    if (openShown)
        openNote_->setText(QString("%1 open surface%2 cannot be capped and are left hollow.")
                               .arg(m.openSurfaces).arg(m.openSurfaces == 1 ? "" : "s"));
    watertightNote_->setVisible(openShown && m.closedSurfaces == 0);
}

SectionWindow::SectionWindow(App& app, QWidget* parent) : QDialog(parent), app_(app) {
    setWindowTitle("Section view");
    setModal(false);
    // Opened from the toolbar while working: it must not take the keyboard
    // away from the viewport.
    setAttribute(Qt::WA_ShowWithoutActivating);
    auto* root = new QVBoxLayout(this);
    controls_ = new SectionControls(app_, this);
    root->addWidget(controls_);
    setFixedWidth(300);
}

void SectionWindow::refresh() {
    const App::SectionModel m = app_.sectionModel();
    if (m.windowOpen != isVisible()) {
        // First shown at the top right of the view rather than centred over
        // the model; after that where it was left.
        if (m.windowOpen && !placed_ && parentWidget()) {
            placed_ = true;
            controls_->refresh(m);
            adjustSize();
            const QWidget* p = parentWidget();
            move(p->mapToGlobal(QPoint(p->width() - width() - 40, 90)));
        }
        setVisible(m.windowOpen);
    }
    if (!m.windowOpen) return;
    controls_->refresh(m);
    adjustSize();
}

void SectionWindow::reject() {
    App* a = &app_;
    app_.post([a] { a->setSectionWindowOpen(false); });
    QDialog::reject();
}

} // namespace shitcad
