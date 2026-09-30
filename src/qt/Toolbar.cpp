#include "Toolbar.h"

#include <QLabel>
#include <QMenu>
#include <QToolButton>

namespace shitcad {

Toolbar::Toolbar(App& app, QWidget* parent) : QToolBar("Toolbar", parent), app_(app) {
    setMovable(false);
    setFloatable(false);
    setToolButtonStyle(Qt::ToolButtonTextOnly);
    setContextMenuPolicy(Qt::PreventContextMenu);
}

QAction* Toolbar::button(const QString& text, UiAction a, int arg) {
    QAction* act = addAction(text);
    App* app = &app_;
    connect(act, &QAction::triggered, this, [app, a, arg] {
        app->post([app, a, arg] { app->perform(a, arg); });
    });
    // Toolbar buttons never take keyboard focus: shortcuts stay with the viewport.
    if (auto* w = widgetForAction(act)) w->setFocusPolicy(Qt::NoFocus);
    return act;
}

void Toolbar::label(const QString& text, bool dim) {
    auto* l = new QLabel(text, this);
    l->setContentsMargins(6, 0, 6, 0);
    if (dim) l->setEnabled(false);
    addWidget(l);
}

void Toolbar::refresh() {
    // States are re-applied every frame, not only when the model changes: a
    // checkable button flips itself when clicked, and a click that changes
    // nothing (the active tool again) must still be put back.
    const ToolbarModel m = app_.toolbarModel();
    if (!built_ || m.variant != last_.variant) rebuild(m);
    apply(m);
    last_ = m;
    built_ = true;
}

void Toolbar::apply(const ToolbarModel& m) {
    for (auto& fn : sync_) fn(m);
}

void Toolbar::rebuild(const ToolbarModel& m) {
    clear();
    sync_.clear();

    auto checkable = [&](QAction* a, std::function<bool(const ToolbarModel&)> on) {
        a->setCheckable(true);
        sync_.push_back([a, on](const ToolbarModel& s) { a->setChecked(on(s)); });
    };
    auto orthoButton = [&] {
        QAction* a = button("[O] Persp", UiAction::ToggleOrtho);
        sync_.push_back([a](const ToolbarModel& s) { a->setText(s.ortho ? "[O]rtho" : "[O] Persp"); });
    };

    if (m.variant == ToolbarModel::Variant::Sketch) {
        button("Finish Sketch [Esc]", UiAction::FinishSketch);
        addSeparator();
        for (int i = 0; i < App::kSketchToolCount; i++) {
            const ToolType t = App::kSketchTools[i];
            checkable(button(App::kSketchToolLabels[i], UiAction::SelectTool, (int)t),
                      [t](const ToolbarModel& s) { return s.tool == t; });
        }
        addSeparator();
        checkable(button("[E]xtrude", UiAction::Extrude), [](const ToolbarModel& s) { return s.extrudeActive; });
        checkable(button("Re[v]olve", UiAction::Revolve), [](const ToolbarModel& s) { return s.revolveActive; });
        checkable(button("Loft", UiAction::Loft), [](const ToolbarModel& s) { return s.loftActive; });
        addSeparator();
        button("[N] Snap View", UiAction::SnapView);
        addSeparator();
        for (int i = 0; i < App::kToolbarConstraintCount; i++) {
            QAction* a = button(App::kToolbarConstraints[i].label, UiAction::ApplyConstraint, i);
            sync_.push_back([a, i](const ToolbarModel& s) { a->setEnabled(s.constraintValid[i]); });
        }
        addSeparator();
        auto* planeLabel = new QLabel(this);
        planeLabel->setContentsMargins(6, 0, 6, 0);
        addWidget(planeLabel);
        sync_.push_back([planeLabel](const ToolbarModel& s) {
            planeLabel->setText(QString("Sketching: %1").arg(QString::fromStdString(s.sketchPlaneName)));
        });
        addSeparator();
        orthoButton();
        button("Export DXF", UiAction::ExportDxf);
        button("Prefs", UiAction::TogglePrefs);
        return;
    }

    // Workspace tabs
    for (Workspace w : {Workspace::Model, Workspace::Simulation}) {
        QAction* a = button(w == Workspace::Model ? "Model" : "Simulation", UiAction::SetWorkspace, (int)w);
        a->setCheckable(true);
        sync_.push_back([a, w](const ToolbarModel& s) {
            a->setChecked(s.workspace == w);
            a->setEnabled(s.canSwitchWorkspace || s.workspace == w);
        });
    }
    addSeparator();

    if (m.variant == ToolbarModel::Variant::Simulation) {
        button("Save", UiAction::Save);
        button("Open", UiAction::Open);
        button("Import STL", UiAction::ImportStl);
        addSeparator();
        label("Set up surfaces and nozzles in the Simulation panel", true);
        addSeparator();
        orthoButton();
        button("Prefs", UiAction::TogglePrefs);
        return;
    }

    button("Save", UiAction::Save);
    button("Open", UiAction::Open);
    auto menuButton = [&](const QString& text, std::initializer_list<std::pair<const char*, UiAction>> items) {
        auto* tb = new QToolButton(this);
        tb->setText(text);
        tb->setPopupMode(QToolButton::InstantPopup);
        tb->setFocusPolicy(Qt::NoFocus);
        auto* menu = new QMenu(tb);
        App* app = &app_;
        for (const auto& it : items) {
            const UiAction a = it.second;
            connect(menu->addAction(it.first), &QAction::triggered, this,
                    [app, a] { app->post([app, a] { app->perform(a); }); });
        }
        tb->setMenu(menu);
        addWidget(tb);
    };
    menuButton("Import", {{"STEP (.step/.stp)", UiAction::ImportStep},
                          {"IGES (.igs/.iges)", UiAction::ImportIges},
                          {"STL (.stl)", UiAction::ImportStl}});
    menuButton("Export", {{"STEP (.step)", UiAction::ExportStep},
                          {"IGES (.igs)", UiAction::ExportIges},
                          {"STL (.stl)", UiAction::ExportStl},
                          {"OBJ (.obj)", UiAction::ExportObj}});
    addSeparator();
    label("Navigate");
    addSeparator();
    checkable(button("Section", UiAction::ToggleSection), [](const ToolbarModel& s) { return s.sectionOn; });
    label("Click a plane or face to sketch", true);
    addSeparator();
    auto* bodies = new QLabel(this);
    bodies->setContentsMargins(6, 0, 6, 0);
    addWidget(bodies);
    sync_.push_back([bodies](const ToolbarModel& s) { bodies->setText(QString("Bodies: %1").arg(s.bodyCount)); });
    addSeparator();
    button("[E]xtrude", UiAction::Extrude);
    button("Re[v]olve", UiAction::Revolve);
    button("Loft", UiAction::Loft);
    addSeparator();
    checkable(button("Union", UiAction::Union), [](const ToolbarModel& s) { return s.unionActive; });
    checkable(button("Subtract", UiAction::Subtract), [](const ToolbarModel& s) { return s.subtractActive; });
    addSeparator();
    orthoButton();
    button("Prefs", UiAction::TogglePrefs);
}

} // namespace shitcad
