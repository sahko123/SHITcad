#include "SimulationPanel.h"
#include "SectionControls.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPainter>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSlider>
#include <QSpinBox>
#include <QTableWidget>
#include <QToolButton>
#include <QVBoxLayout>

#include <array>
#include <cmath>

namespace shitcad {

// ---- helpers -------------------------------------------------------------

namespace {

QLabel* note(const QString& text, QWidget* parent, const char* colour = nullptr) {
    auto* label = new QLabel(text, parent);
    label->setWordWrap(true);
    if (colour) label->setStyleSheet(QString("color: %1;").arg(colour));
    else label->setEnabled(false);   // greyed, like ImGui::TextDisabled
    return label;
}

// A collapsible section: a header button that shows and hides its body.
QWidget* section(const QString& title, QWidget* body, bool open, QWidget* parent) {
    auto* box = new QWidget(parent);
    auto* col = new QVBoxLayout(box);
    col->setContentsMargins(0, 0, 0, 0);
    col->setSpacing(2);
    auto* head = new QToolButton(box);
    head->setText(title);
    head->setCheckable(true);
    head->setChecked(open);
    head->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    head->setArrowType(open ? Qt::DownArrow : Qt::RightArrow);
    head->setAutoRaise(true);
    head->setFocusPolicy(Qt::NoFocus);
    QFont f = head->font();
    f.setBold(true);
    head->setFont(f);
    col->addWidget(head);
    body->setParent(box);
    body->setVisible(open);
    col->addWidget(body);
    QObject::connect(head, &QToolButton::toggled, box, [head, body](bool on) {
        head->setArrowType(on ? Qt::DownArrow : Qt::RightArrow);
        body->setVisible(on);
    });
    return box;
}

void setSpin(QDoubleSpinBox* s, double v) {
    if (s->hasFocus() || s->value() == v) return;
    const QSignalBlocker block(s);
    s->setValue(v);
}

void setSpin(QSpinBox* s, int v) {
    if (s->hasFocus() || s->value() == v) return;
    const QSignalBlocker block(s);
    s->setValue(v);
}

void setText(QLineEdit* e, const std::string& text) {
    const QString t = QString::fromStdString(text);
    if (e->hasFocus() || e->text() == t) return;
    const QSignalBlocker block(e);
    e->setText(t);
}

void setLabel(QLabel* l, const QString& text) {
    if (l->text() != text) l->setText(text);
}

QDoubleSpinBox* spin(QWidget* parent, double lo, double hi, int decimals) {
    auto* s = new QDoubleSpinBox(parent);
    s->setRange(lo, hi);
    s->setDecimals(decimals);
    s->setKeyboardTracking(false);   // valueChanged on Enter, focus loss or a step
    s->setButtonSymbols(QAbstractSpinBox::NoButtons);
    return s;
}

QColor rgb(const float c[3]) { return QColor::fromRgbF(c[0], c[1], c[2]); }

} // namespace

// Colour legend for the results field: swatches for a categorical field, a
// ramp for a continuous one.
class ResultLegend : public QWidget {
public:
    explicit ResultLegend(QWidget* parent) : QWidget(parent) {}

    void setModel(const App::SimResultsModel& m) {
        m_ = m;
        const int line = fontMetrics().height() + 4;
        int rows = m.categorical ? (int)m.categories.size() + (m.reachNote ? 2 : 0)
                                 : 3 + (m.flux ? 1 : 0) + (m.flux && !m.fluxTrustworthy ? 1 : 0) + (m.noData > 0 ? 1 : 0);
        setFixedHeight(rows * line + 4);
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter p(this);
        const QFontMetrics fm = fontMetrics();
        const int line = fm.height() + 4, sw = fm.height();
        const QColor text = palette().color(QPalette::WindowText);
        const QColor grey = palette().color(QPalette::Disabled, QPalette::WindowText);
        int y = 2;
        auto swatchRow = [&](const QColor& c, const QString& label, const QColor& tc) {
            p.fillRect(0, y, sw, sw, c);
            p.setPen(tc);
            p.drawText(sw + 6, y, width() - sw - 6, sw, Qt::AlignVCenter, label);
            y += line;
        };
        auto textRow = [&](const QString& s, const QColor& tc) {
            p.setPen(tc);
            p.drawText(0, y, width(), sw, Qt::AlignVCenter, s);
            y += line;
        };
        if (m_.categorical) {
            for (const auto& s : m_.categories) swatchRow(rgb(s.rgb), QString::fromStdString(s.label), text);
            if (m_.reachNote) {
                textRow("  percentages are of scored wall area;", grey);
                textRow("  caps and obstructions are drawn muted", grey);
            }
            return;
        }
        const int steps = 32;
        for (int s = 0; s < steps; s++) {
            float a[3];
            rampColour((float)s / steps, a);
            const int x0 = width() * s / steps, x1 = width() * (s + 1) / steps;
            p.fillRect(x0, y, x1 - x0, sw, rgb(a));
        }
        y += line;
        p.setPen(text);
        p.drawText(0, y, width(), sw, Qt::AlignLeft | Qt::AlignVCenter, "0");
        p.drawText(0, y, width(), sw, Qt::AlignRight | Qt::AlignVCenter,
                   QString::asprintf("%.3g %s", m_.hi, m_.unit.c_str()));
        y += line;
        swatchRow(rgb(kNoValueColour), "none reached (scale tops out at the 95th percentile)", grey);
        if (m_.flux) {
            swatchRow(rgb(kUnsampledColour), "sprayed, but no ray sampled it - raise rays", grey);
            if (!m_.fluxTrustworthy)
                textRow("Flux map under-sampled: raise rays. Coverage is unaffected.", QColor(176, 128, 0));
        }
        if (m_.noData > 0) textRow(QString("%1 faces have no value for this field").arg(m_.noData), grey);
    }

private:
    App::SimResultsModel m_;
};

// ---- construction --------------------------------------------------------

SimulationPanel::SimulationPanel(App& app, QWidget* parent) : QDockWidget("Simulation", parent), app_(app) {
    setObjectName("simulation");   // saveState() keys docks by name
    setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable);
    App* a = &app_;

    auto* content = new QWidget;
    auto* col = new QVBoxLayout(content);
    col->setContentsMargins(6, 6, 6, 6);
    col->addWidget(note("Tier 1: spray line of sight + splash", content));
    col->addWidget(section("Surfaces", buildSurfaces(), true, content));
    col->addWidget(section("Nozzles", buildNozzles(), true, content));
    col->addWidget(section("Run settings", buildRunSettings(), false, content));
    viewBody_ = new QWidget;
    auto* viewCol = new QVBoxLayout(viewBody_);
    viewCol->setContentsMargins(0, 0, 0, 0);
    section_ = new SectionControls(app_, viewBody_);
    viewCol->addWidget(section_);
    col->addWidget(section("View", viewBody_, false, content));
    col->addWidget(section("Run", buildRun(), true, content));
    results_ = section("Results", buildResults(), true, content);
    col->addWidget(results_);
    auto* exportSpec = new QPushButton("Export spec...", content);
    exportSpec->setFocusPolicy(Qt::NoFocus);
    connect(exportSpec, &QPushButton::clicked, this, [a] { a->post([a] { a->exportSimulationSpecDialog(); }); });
    col->addWidget(exportSpec);
    message_ = new QLabel(content);
    message_->setWordWrap(true);
    col->addWidget(message_);
    col->addStretch(1);

    auto* scroll = new QScrollArea(this);
    scroll->setWidgetResizable(true);
    scroll->setWidget(content);
    scroll->setFrameShape(QFrame::NoFrame);
    setWidget(scroll);
    setMinimumWidth(330);
    hide();
}

QWidget* SimulationPanel::buildSurfaces() {
    auto* body = new QWidget;
    auto* col = new QVBoxLayout(body);
    col->setContentsMargins(0, 0, 0, 0);
    auto* rows = new QWidget(body);
    surfaceRows_ = new QVBoxLayout(rows);
    surfaceRows_->setContentsMargins(0, 0, 0, 0);
    col->addWidget(rows);
    surfaceNote_ = note("", body);
    col->addWidget(surfaceNote_);
    return body;
}

QWidget* SimulationPanel::buildNozzles() {
    auto* body = new QWidget;
    auto* col = new QVBoxLayout(body);
    col->setContentsMargins(0, 0, 0, 0);
    App* a = &app_;

    place_ = new QPushButton(body);
    place_->setFocusPolicy(Qt::NoFocus);
    connect(place_, &QPushButton::clicked, this, [this, a] {
        const bool placing = place_->property("placing").toBool();
        a->post([a, placing] { a->setNozzlePlacing(!placing); });
    });
    col->addWidget(place_);
    auto* standoffRow = new QHBoxLayout;
    standoff_ = spin(body, 0.0, 1e6, 1);
    standoff_->setMaximumWidth(90);
    connect(standoff_, &QDoubleSpinBox::valueChanged, this, [a](double v) {
        a->post([a, v] { a->setNozzleStandoff((float)v); });
    });
    standoffRow->addWidget(standoff_);
    standoffRow->addWidget(new QLabel("Standoff from surface (mm)", body), 1);
    col->addLayout(standoffRow);
    col->addWidget(note("Shift-click places several. Click a cone's apex to select.", body));

    nozzles_ = new QListWidget(body);
    nozzles_->setMaximumHeight(110);
    connect(nozzles_, &QListWidget::currentRowChanged, this, [this, a](int row) {
        if (row < 0) return;
        const uint32_t id = nozzles_->item(row)->data(Qt::UserRole).toUInt();
        a->post([a, id] { a->selectNozzle(id); });
    });
    col->addWidget(nozzles_);

    detail_ = new QWidget(body);
    auto* d = new QVBoxLayout(detail_);
    d->setContentsMargins(0, 4, 0, 0);
    auto* nameRow = new QHBoxLayout;
    name_ = new QLineEdit(detail_);
    connect(name_, &QLineEdit::editingFinished, this, [this, a] {
        const uint32_t id = detailId_;
        const std::string s = name_->text().toUtf8().toStdString();
        a->post([a, id, s] { a->setNozzleName(id, s); });
    });
    nameRow->addWidget(name_, 1);
    nameRow->addWidget(new QLabel("Name", detail_));
    d->addLayout(nameRow);

    placement_ = new QWidget(detail_);
    auto* pl = new QVBoxLayout(placement_);
    pl->setContentsMargins(0, 0, 0, 0);
    auto* posRow = new QHBoxLayout;
    auto* axisRow = new QHBoxLayout;
    for (int k = 0; k < 3; k++) {
        pos_[k] = spin(placement_, -1e7, 1e7, 1);
        connect(pos_[k], &QDoubleSpinBox::valueChanged, this, [this] { pushPosition(); });
        posRow->addWidget(pos_[k]);
        axis_[k] = spin(placement_, -1e3, 1e3, 3);
        connect(axis_[k], &QDoubleSpinBox::valueChanged, this, [this] { pushAxis(); });
        axisRow->addWidget(axis_[k]);
    }
    posRow->addWidget(new QLabel("Pos mm", placement_));
    axisRow->addWidget(new QLabel("Axis", placement_));
    pl->addLayout(posRow);
    pl->addLayout(axisRow);
    auto* flipRow = new QHBoxLayout;
    auto* flip = new QPushButton("Flip", placement_);
    auto* down = new QPushButton("Point down (-Y)", placement_);
    for (auto* b : {flip, down}) b->setFocusPolicy(Qt::NoFocus);
    connect(flip, &QPushButton::clicked, this, [this, a] { const uint32_t id = detailId_; a->post([a, id] { a->flipNozzle(id); }); });
    connect(down, &QPushButton::clicked, this, [this, a] { const uint32_t id = detailId_; a->post([a, id] { a->pointNozzleDown(id); }); });
    flipRow->addWidget(flip);
    flipRow->addWidget(down);
    flipRow->addStretch(1);
    pl->addLayout(flipRow);
    d->addWidget(placement_);
    orphan_ = note("Its surface was deleted.", detail_, "#d05050");
    d->addWidget(orphan_);

    auto* angleRow = new QHBoxLayout;
    halfAngle_ = new QSlider(Qt::Horizontal, detail_);
    halfAngle_->setRange(1, 180);
    halfAngleText_ = new QLabel(detail_);
    halfAngleText_->setMinimumWidth(80);
    connect(halfAngle_, &QSlider::valueChanged, this, [this, a](int v) {
        const uint32_t id = detailId_;
        halfAngleText_->setText(QString("%1 deg  Half-angle").arg(v));
        a->post([a, id, v] { a->setNozzleHalfAngle(id, (float)v); });
        if (!halfAngle_->isSliderDown()) a->post([a] { a->commitSimulationEdit(); });   // keyboard / click
    });
    connect(halfAngle_, &QSlider::sliderReleased, this, [a] { a->post([a] { a->commitSimulationEdit(); }); });
    angleRow->addWidget(halfAngle_, 1);
    angleRow->addWidget(halfAngleText_);
    d->addLayout(angleRow);

    auto* flowRow = new QHBoxLayout;
    flow_ = spin(detail_, 0.001, 1e4, 3);
    connect(flow_, &QDoubleSpinBox::valueChanged, this, [this, a](double v) {
        const uint32_t id = detailId_;
        a->post([a, id, v] { a->setNozzleFlow(id, (float)v); a->commitSimulationEdit(); });
    });
    flowRow->addWidget(flow_, 1);
    flowRow->addWidget(new QLabel("Flow kg/s", detail_));
    d->addLayout(flowRow);
    auto* pressRow = new QHBoxLayout;
    pressure_ = spin(detail_, -1e4, 1e4, 2);
    connect(pressure_, &QDoubleSpinBox::valueChanged, this, [this, a](double v) {
        const uint32_t id = detailId_;
        a->post([a, id, v] { a->setNozzlePressure(id, (float)v); a->commitSimulationEdit(); });
    });
    pressRow->addWidget(pressure_, 1);
    pressRow->addWidget(new QLabel("Press. bar", detail_));
    d->addLayout(pressRow);
    d->addWidget(note("Half-angle 180 = full spray ball. Pressure is for CFD; Tier 1 uses flow.", detail_));
    auto* del = new QPushButton("Delete nozzle", detail_);
    del->setFocusPolicy(Qt::NoFocus);
    connect(del, &QPushButton::clicked, this, [this, a] { const uint32_t id = detailId_; a->post([a, id] { a->deleteNozzle(id); }); });
    d->addWidget(del);
    col->addWidget(detail_);
    return body;
}

void SimulationPanel::pushPosition() {
    App* a = &app_;
    const uint32_t id = detailId_;
    const std::array<double, 3> p = {pos_[0]->value(), pos_[1]->value(), pos_[2]->value()};
    a->post([a, id, p] { a->setNozzlePosition(id, p.data()); a->commitSimulationEdit(); });
}

void SimulationPanel::pushAxis() {
    App* a = &app_;
    const uint32_t id = detailId_;
    const std::array<double, 3> v = {axis_[0]->value(), axis_[1]->value(), axis_[2]->value()};
    a->post([a, id, v] { a->setNozzleAxis(id, v.data()); a->commitSimulationEdit(); });
}

QWidget* SimulationPanel::buildRunSettings() {
    auto* body = new QWidget;
    auto* col = new QVBoxLayout(body);
    col->setContentsMargins(0, 0, 0, 0);
    App* a = &app_;
    auto* raysRow = new QHBoxLayout;
    rays_ = new QSpinBox(body);
    rays_->setRange(1000, 5000000);
    rays_->setSingleStep(10000);
    rays_->setKeyboardTracking(false);
    connect(rays_, &QSpinBox::valueChanged, this, [a](int v) {
        a->post([a, v] { a->setSimulationRays(v); a->commitSimulationEdit(); });
    });
    raysRow->addWidget(rays_);
    raysRow->addWidget(new QLabel("Rays per nozzle", body), 1);
    col->addLayout(raysRow);
    auto* bounceRow = new QHBoxLayout;
    bounces_ = new QSpinBox(body);
    bounces_->setRange(0, 4);
    bounces_->setKeyboardTracking(false);
    connect(bounces_, &QSpinBox::valueChanged, this, [a](int v) {
        a->post([a, v] { a->setSimulationBounces(v); a->commitSimulationEdit(); });
    });
    bounceRow->addWidget(bounces_);
    bounceRow->addWidget(new QLabel("Splash bounces", body), 1);
    col->addLayout(bounceRow);
    col->addWidget(note("Coverage is exact at any ray count; rays only sharpen the flux map.", body));
    return body;
}

QWidget* SimulationPanel::buildRun() {
    auto* body = new QWidget;
    auto* col = new QVBoxLayout(body);
    col->setContentsMargins(0, 0, 0, 0);
    App* a = &app_;

    auto* engine = new QWidget;
    auto* e = new QVBoxLayout(engine);
    e->setContentsMargins(8, 0, 0, 0);
    e->addWidget(new QLabel("cip-sim folder", engine));
    auto* pathRow = new QHBoxLayout;
    cipSim_ = new QLineEdit(engine);
    auto* browse = new QPushButton("Browse", engine);
    browse->setFocusPolicy(Qt::NoFocus);
    pathRow->addWidget(cipSim_, 1);
    pathRow->addWidget(browse);
    e->addLayout(pathRow);
    e->addWidget(new QLabel("Python", engine));
    python_ = new QLineEdit(engine);
    e->addWidget(python_);
    engineNote_ = new QLabel(engine);
    engineNote_->setWordWrap(true);
    e->addWidget(engineNote_);
    auto pushPaths = [this, a] {
        const std::string p = cipSim_->text().toUtf8().toStdString();
        const std::string py = python_->text().toUtf8().toStdString();
        a->post([a, p, py] { a->setEnginePaths(p, py); });
    };
    connect(cipSim_, &QLineEdit::editingFinished, this, pushPaths);
    connect(python_, &QLineEdit::editingFinished, this, pushPaths);
    connect(browse, &QPushButton::clicked, this, [a] { a->post([a] { a->browseEngineFolder(); }); });
    col->addWidget(section("Engine (cip-sim)", engine, false, body));

    runStatus_ = new QLabel(body);
    runStatus_->setWordWrap(true);
    col->addWidget(runStatus_);
    runButton_ = new QPushButton("Run Tier 1 coverage", body);
    runButton_->setStyleSheet("QPushButton:enabled { background: #33803f; color: white; }");
    cancelButton_ = new QPushButton("Cancel", body);
    for (auto* b : {runButton_, cancelButton_}) b->setFocusPolicy(Qt::NoFocus);
    connect(runButton_, &QPushButton::clicked, this, [a] { a->post([a] { a->startTier1Run(); }); });
    connect(cancelButton_, &QPushButton::clicked, this, [a] { a->post([a] { a->cancelSimulationRun(); }); });
    col->addWidget(runButton_);
    col->addWidget(cancelButton_);
    runError_ = note("", body, "#d05050");
    col->addWidget(runError_);
    log_ = new QPlainTextEdit;
    log_->setReadOnly(true);
    log_->setMaximumHeight(140);
    col->addWidget(section("Log", log_, false, body));
    return body;
}

QWidget* SimulationPanel::buildResults() {
    auto* body = new QWidget;
    auto* col = new QVBoxLayout(body);
    col->setContentsMargins(0, 0, 0, 0);
    App* a = &app_;
    stale_ = note("Set-up, settings or geometry changed since this run - run again.", body, "#b08000");
    col->addWidget(stale_);
    showResults_ = new QCheckBox("Show on geometry", body);
    showResults_->setFocusPolicy(Qt::NoFocus);
    connect(showResults_, &QCheckBox::clicked, this, [a](bool on) { a->post([a, on] { a->setResultsShown(on); }); });
    col->addWidget(showResults_);
    field_ = new QComboBox(body);
    connect(field_, &QComboBox::activated, this, [a](int i) { a->post([a, i] { a->setResultsField(i); }); });
    col->addWidget(field_);
    legend_ = new ResultLegend(body);
    col->addWidget(legend_);
    coverage_ = new QTableWidget(0, 5, body);
    coverage_->setHorizontalHeaderLabels({"Surface", "Direct", "Splash", "Dry", "m2"});
    coverage_->verticalHeader()->hide();
    coverage_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    coverage_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    coverage_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    coverage_->setSelectionMode(QAbstractItemView::NoSelection);
    coverage_->setFocusPolicy(Qt::NoFocus);
    col->addWidget(coverage_);
    col->addWidget(note("Tier 1 is line of sight + range-limited splash, not CFD.", body));
    auto* buttons = new QHBoxLayout;
    auto* paraview = new QPushButton("Open in ParaView", body);
    auto* folder = new QPushButton("Open run folder", body);
    for (auto* b : {paraview, folder}) { b->setFocusPolicy(Qt::NoFocus); buttons->addWidget(b); }
    buttons->addStretch(1);
    connect(paraview, &QPushButton::clicked, this, [a] { a->post([a] { a->openResultsInParaView(); }); });
    connect(folder, &QPushButton::clicked, this, [a] { a->post([a] { a->openRunFolder(); }); });
    col->addLayout(buttons);
    paraview_ = note("", body);
    col->addWidget(paraview_);
    return body;
}

// ---- refresh -------------------------------------------------------------

void SimulationPanel::refresh() {
    const App::SimSetupModel setup = app_.simSetupModel();
    if (setup.open == isHidden()) setVisible(setup.open);
    if (!setup.open) return;
    refreshSetup(setup);
    refreshRun(app_.simRunModel());
    refreshResults(app_.simResultsModel());
    if (viewBody_->isVisible()) section_->refresh(app_.sectionModel());
}

void SimulationPanel::refreshSetup(const App::SimSetupModel& m) {
    App* a = &app_;

    // Surfaces: rows rebuilt when the set of imports changes
    std::string key;
    for (const auto& s : m.surfaces)
        key += std::to_string(s.feature) + ":" + s.name + (s.active ? "+" : "-") + (s.failed ? "!" + s.errorMsg : "") + "\n";
    if (key != shownSurfaces_) {
        shownSurfaces_ = key;
        roleCombos_.clear();
        while (QLayoutItem* item = surfaceRows_->takeAt(0)) {
            delete item->widget();
            delete item;
        }
        for (const auto& s : m.surfaces) {
            auto* row = new QWidget;
            auto* rc = new QVBoxLayout(row);
            rc->setContentsMargins(0, 0, 0, 0);
            auto* h = new QHBoxLayout;
            auto* name = new QLabel(QString::fromStdString(s.name), row);
            name->setEnabled(s.active);
            h->addWidget(name, 1);
            auto* role = new QComboBox(row);
            for (int r = 0; r < kSurfaceRoleCount; r++) role->addItem(surfaceRoleLabel((SurfaceRole)r));
            const uint32_t feature = s.feature;
            connect(role, &QComboBox::activated, this, [a, feature](int r) { a->post([a, feature, r] { a->setSurfaceRole(feature, r); }); });
            h->addWidget(role, 1);
            rc->addLayout(h);
            if (s.failed) rc->addWidget(note(QString("  left out: %1").arg(QString::fromStdString(s.errorMsg)), row, "#d05050"));
            else if (!s.active) rc->addWidget(note("  left out: suppressed or rolled back", row));
            surfaceRows_->addWidget(row);
            roleCombos_.push_back(role);
        }
    }
    for (size_t i = 0; i < roleCombos_.size() && i < m.surfaces.size(); i++) {
        if (roleCombos_[i]->currentIndex() != m.surfaces[i].role) {
            const QSignalBlocker block(roleCombos_[i]);
            roleCombos_[i]->setCurrentIndex(m.surfaces[i].role);
        }
    }
    setLabel(surfaceNote_, m.surfaces.empty()
        ? "No surfaces yet. Import the vessel with Import > STL, one file per surface (wall, inlet cap, drain cap...)."
        : "Walls count toward coverage; caps and obstructions only block spray.");

    // Nozzles
    place_->setProperty("placing", m.placing);
    const QString placeText = m.placing ? "Click a surface to place... (Esc)" : "+ Place nozzle";
    if (place_->text() != placeText) {
        place_->setText(placeText);
        place_->setStyleSheet(m.placing ? "background: #33993f; color: white;" : "");
    }
    setSpin(standoff_, m.standoffMm);

    std::string nkey;
    for (const auto& n : m.nozzles) nkey += std::to_string(n.id) + ":" + n.label + "\n";
    if (nkey != shownNozzles_) {
        shownNozzles_ = nkey;
        const QSignalBlocker block(nozzles_);
        nozzles_->clear();
        for (const auto& n : m.nozzles) {
            auto* item = new QListWidgetItem(QString::fromStdString(n.label), nozzles_);
            item->setData(Qt::UserRole, n.id);
        }
    }
    {
        int row = -1;
        for (int i = 0; i < (int)m.nozzles.size(); i++)
            if (m.nozzles[i].id == m.selected) row = i;
        if (nozzles_->currentRow() != row) {
            const QSignalBlocker block(nozzles_);
            nozzles_->setCurrentRow(row);
        }
    }

    detail_->setVisible(m.selected != 0);
    if (m.selected == 0) return;
    if (detailId_ != m.selected) {
        detailId_ = m.selected;
        // A different nozzle: nothing being typed belongs to it.
        for (QWidget* w : {(QWidget*)name_, (QWidget*)flow_, (QWidget*)pressure_}) w->clearFocus();
    }
    setText(name_, m.name);
    placement_->setVisible(m.hosted);
    orphan_->setVisible(!m.hosted);
    for (int k = 0; k < 3; k++) {
        setSpin(pos_[k], m.pos[k]);
        setSpin(axis_[k], m.axis[k]);
    }
    if (!halfAngle_->isSliderDown()) {
        const int v = (int)std::lround(m.halfAngleDeg);
        if (halfAngle_->value() != v) {
            const QSignalBlocker block(halfAngle_);
            halfAngle_->setValue(v);
        }
        setLabel(halfAngleText_, QString("%1 deg  Half-angle").arg(v));
    }
    setSpin(flow_, m.flowKgS);
    setSpin(pressure_, m.pressureBar);
    setSpin(rays_, m.rays);
    setSpin(bounces_, m.bounces);

    // Export / validation message
    QString msg;
    if (!m.message.empty()) {
        msg = QString("<span style='color:%1'>%2</span>")
                  .arg(m.messageIsError ? "#d05050" : "#2e8b30", QString::fromStdString(m.message).toHtmlEscaped());
        for (const auto& w : m.warnings)
            msg += QString("<br><span style='color:#b08000'>%1</span>").arg(QString::fromStdString(w).toHtmlEscaped());
    }
    message_->setVisible(!msg.isEmpty());
    setLabel(message_, msg);
}

void SimulationPanel::refreshRun(const App::SimRunModel& m) {
    setText(cipSim_, m.cipSimPath);
    setText(python_, m.python);
    if (!m.problem.empty()) {
        setLabel(engineNote_, QString::fromStdString(m.problem));
        engineNote_->setStyleSheet("color: #b08000;");
        engineNote_->setEnabled(true);
    } else {
        setLabel(engineNote_, "Saved for this computer, not in the project.");
        engineNote_->setStyleSheet("");
        engineNote_->setEnabled(false);
    }

    QString status;
    if (m.running) {
        status = QString::asprintf("Running Tier 1... %.0f s", m.seconds);
        if (!m.log.empty()) status += "\n" + QString::fromStdString(m.log.back());
    } else if (m.done) {
        status = QString::asprintf("Finished in %.0f s", m.seconds);
    } else if (m.cancelled) {
        status = "Cancelled.";
    }
    runStatus_->setVisible(!status.isEmpty());
    setLabel(runStatus_, status);
    runStatus_->setStyleSheet(m.done ? "color: #2e8b30;" : "");
    runButton_->setVisible(!m.running);
    runButton_->setEnabled(m.problem.empty());
    cancelButton_->setVisible(m.running);
    runError_->setVisible(!m.error.empty());
    setLabel(runError_, QString::fromStdString(m.error));
    if (m.log.size() != shownLog_) {
        shownLog_ = m.log.size();
        QStringList lines;
        for (const auto& l : m.log) lines << QString::fromStdString(l);
        log_->setPlainText(lines.join('\n'));
    }
}

void SimulationPanel::refreshResults(const App::SimResultsModel& m) {
    results_->setVisible(m.loaded);
    if (!m.loaded) return;
    stale_->setVisible(m.stale);
    if (showResults_->isChecked() != m.show) {
        const QSignalBlocker block(showResults_);
        showResults_->setChecked(m.show);
    }
    if (m.fieldLabels != shownFields_) {
        shownFields_ = m.fieldLabels;
        const QSignalBlocker block(field_);
        field_->clear();
        for (const auto& f : m.fieldLabels) field_->addItem(QString::fromStdString(f));
    }
    if (field_->currentIndex() != m.field) {
        const QSignalBlocker block(field_);
        field_->setCurrentIndex(m.field);
    }

    // Legend and table change only with a new run or field
    std::string key = std::to_string(m.field) + "|" + std::to_string(m.fieldLabels.size()) + "|" +
                      std::to_string(m.overall.areaM2) + "|" + std::to_string(m.overall.dryPct) + "|" +
                      std::to_string(m.surfaces.size());
    if (key != shownCoverage_) {
        shownCoverage_ = key;
        legend_->setModel(m);
        std::vector<std::pair<const CoverageRow*, bool>> rows = {{&m.overall, true}};
        for (const auto& r : m.surfaces) rows.push_back({&r, false});
        coverage_->setRowCount((int)rows.size());
        for (int i = 0; i < (int)rows.size(); i++) {
            const CoverageRow& r = *rows[i].first;
            const bool overall = rows[i].second;
            auto cell = [&](int c, const QString& t, const QColor* colour = nullptr) {
                auto* item = new QTableWidgetItem(t);
                if (colour) item->setForeground(*colour);
                coverage_->setItem(i, c, item);
            };
            const QColor grey = palette().color(QPalette::Disabled, QPalette::WindowText);
            const QColor dry(208, 80, 70);
            QString name = QString::fromStdString(r.name);
            if (!overall && !r.scored) name += " (cap)";
            cell(0, name, (!overall && !r.scored) ? &grey : nullptr);
            cell(1, QString::asprintf("%.1f%%", r.directPct));
            cell(2, QString::asprintf("%.1f%%", r.splashPct));
            cell(3, QString::asprintf("%.1f%%", r.dryPct), r.dryPct > 0.05 ? &dry : nullptr);
            cell(4, QString::asprintf("%.3g", r.areaM2));
        }
        coverage_->setFixedHeight(coverage_->horizontalHeader()->height() +
                                  coverage_->rowCount() * coverage_->verticalHeader()->defaultSectionSize() + 4);
    }
    paraview_->setVisible(!m.paraviewMessage.empty());
    setLabel(paraview_, QString::fromStdString(m.paraviewMessage));
}

} // namespace shitcad
