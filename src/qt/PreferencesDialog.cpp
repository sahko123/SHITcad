#include "PreferencesDialog.h"

#include <QCheckBox>
#include <QColorDialog>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include <cmath>

namespace shitcad {

namespace {

QColor toQColor(const float* c, bool alpha) {
    auto ch = [](float v) { return (int)std::lround(std::fmin(std::fmax(v, 0.0f), 1.0f) * 255.0f); };
    return QColor(ch(c[0]), ch(c[1]), ch(c[2]), alpha ? ch(c[3]) : 255);
}

void setSwatch(QPushButton* b, const float* c, bool alpha) {
    const QColor q = toQColor(c, alpha);
    b->setStyleSheet(QString("QPushButton { background-color: rgba(%1,%2,%3,%4); border: 1px solid #888; min-width: 40px; }")
                         .arg(q.red()).arg(q.green()).arg(q.blue()).arg(q.alpha()));
}

QDoubleSpinBox* spin(double lo, double hi, double step, int decimals) {
    auto* s = new QDoubleSpinBox;
    s->setRange(lo, hi);
    s->setSingleStep(step);
    s->setDecimals(decimals);
    s->setKeyboardTracking(false); // apply on Enter / arrows / focus-out, not per keystroke
    return s;
}

} // namespace

PreferencesDialog::PreferencesDialog(App& app, QWidget* parent) : QDialog(parent), app_(app) {
    setWindowTitle("Preferences");
    setModal(false);
    shown_ = app_.preferences();

    auto* root = new QVBoxLayout(this);
    auto group = [&](const QString& title) {
        auto* box = new QGroupBox(title, this);
        auto* form = new QFormLayout(box);
        root->addWidget(box);
        return form;
    };

    {
        auto* form = group("Display");
        lightMode_ = new QCheckBox("Light Mode");
        showEdges_ = new QCheckBox("Show Edges");
        form->addRow(lightMode_);
        form->addRow(showEdges_);
        connect(lightMode_, &QCheckBox::toggled, this, [this] { push(); });
        connect(showEdges_, &QCheckBox::toggled, this, [this] { push(); });
    }
    {
        auto* form = group("Sketch Lines");
        form->addRow("Line Color", colourButton(shown_.sketchLineColor, false, "Sketch line colour"));
        lineWidth_ = spin(0.5, 5.0, 0.1, 1);
        form->addRow("Line Width", lineWidth_);
    }
    {
        auto* form = group("Body Edges");
        form->addRow("Edge Color", colourButton(shown_.edgeColor, false, "Edge colour"));
        edgeWidth_ = spin(0.5, 5.0, 0.1, 1);
        form->addRow("Edge Width", edgeWidth_);
    }
    {
        auto* form = group("Dimension Labels");
        form->addRow("Dim Line", colourButton(shown_.dimLineCol, true, "Dimension line colour"));
        form->addRow("Dim Text", colourButton(shown_.dimTextCol, true, "Dimension text colour"));
        form->addRow("Dim Bg", colourButton(shown_.dimBgCol, true, "Dimension background"));
    }
    {
        auto* form = group("Constraint Labels");
        form->addRow("Con Text", colourButton(shown_.conTextCol, true, "Constraint label text"));
        form->addRow("Con Bg", colourButton(shown_.conBgCol, true, "Constraint label background"));
    }
    {
        auto* form = group("Snapping");
        tangentSnap_ = spin(5.0, 40.0, 1.0, 0);
        form->addRow("Tangent Snap (px)", tangentSnap_);
    }
    {
        auto* form = group("Profile Detection");
        backend_ = new QComboBox;
        backend_->addItem("Custom (half-edge tracer)");
        backend_->addItem("OCCT (exact geometry)");
        form->addRow("Backend", backend_);
        backendNote_ = new QLabel("Uses OCCT's BOPAlgo_BuilderFace for exact curve intersections. "
                                  "Ellipses and splines produce smooth edges instead of polyline approximations.");
        backendNote_->setWordWrap(true);
        form->addRow(backendNote_);
        connect(backend_, &QComboBox::currentIndexChanged, this, [this] { push(); });
    }
    root->addStretch(1);

    for (QDoubleSpinBox* s : {lineWidth_, edgeWidth_, tangentSnap_})
        connect(s, &QDoubleSpinBox::valueChanged, this, [this] { push(); });

    load(shown_);
}

QPushButton* PreferencesDialog::colourButton(float* rgba, bool alpha, const QString& title) {
    auto* b = new QPushButton(this);
    b->setFlat(false);
    swatches_.push_back({b, rgba, alpha});
    connect(b, &QPushButton::clicked, this, [this, rgba, alpha, title] {
        QColorDialog::ColorDialogOptions opts;
        if (alpha) opts |= QColorDialog::ShowAlphaChannel;
        const QColor c = QColorDialog::getColor(toQColor(rgba, alpha), this, title, opts);
        if (!c.isValid()) return;
        rgba[0] = (float)c.redF(); rgba[1] = (float)c.greenF(); rgba[2] = (float)c.blueF();
        if (alpha) rgba[3] = (float)c.alphaF();
        push();
    });
    return b;
}

void PreferencesDialog::load(const Preferences& p) {
    // shown_ is overwritten, which the swatch pointers point into.
    shown_ = p;
    const QSignalBlocker b1(lightMode_), b2(showEdges_), b3(lineWidth_), b4(edgeWidth_),
        b5(tangentSnap_), b6(backend_);
    lightMode_->setChecked(p.lightMode);
    showEdges_->setChecked(p.showWireframe);
    if (!lineWidth_->hasFocus()) lineWidth_->setValue(p.sketchLineThickness);
    if (!edgeWidth_->hasFocus()) edgeWidth_->setValue(p.edgeThickness);
    if (!tangentSnap_->hasFocus()) tangentSnap_->setValue(p.tangentSnapPx);
    backend_->setCurrentIndex((int)p.profileBackend);
    backendNote_->setVisible(p.profileBackend == ProfileDetectorBackend::OCCT);
    for (const auto& s : swatches_) setSwatch(s.button, s.value, s.alpha);
    loaded_ = true;
}

void PreferencesDialog::push() {
    Preferences p = shown_; // colours were edited in place
    p.lightMode = lightMode_->isChecked();
    p.showWireframe = showEdges_->isChecked();
    p.sketchLineThickness = (float)lineWidth_->value();
    p.edgeThickness = (float)edgeWidth_->value();
    p.tangentSnapPx = (float)tangentSnap_->value();
    p.profileBackend = (ProfileDetectorBackend)backend_->currentIndex();
    shown_ = p;
    backendNote_->setVisible(p.profileBackend == ProfileDetectorBackend::OCCT);
    for (const auto& s : swatches_) setSwatch(s.button, s.value, s.alpha);
    App* app = &app_;
    app_.post([app, p] { app->setPreferences(p); });
}

void PreferencesDialog::refresh() {
    const bool open = app_.preferencesOpen();
    if (open != isVisible()) setVisible(open);
    if (!open) return;
    // Pull App's values when they differ from what the dialog shows. A push
    // that is still queued looks like a difference for one frame, so wait
    // until nothing is pending (App compares equal to the last push, or the
    // push has run and App changed it further, e.g. light mode's reset).
    const Preferences& now = app_.preferences();
    if (now != shown_ && !app_.hasPosted()) load(now);
}

void PreferencesDialog::reject() {
    App* app = &app_;
    app_.post([app] { app->setPreferencesOpen(false); });
    QDialog::reject();
}

} // namespace shitcad
