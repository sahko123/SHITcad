#pragma once
#include "App.h"

#include <QToolBar>

#include <functional>
#include <vector>

#undef near
#undef far

namespace shitcad {

// Qt front end for ToolbarModel: the buttons for the current mode
// (App::drawToolbar), driven by the same model and actions. refresh() runs
// after every frame; it rebuilds the bar when the variant (sketch / model /
// simulation) changes and otherwise only updates checked, enabled and text
// states in place. Clicks are posted to App, so they run at the start of the
// next frame with the GL context current.
class Toolbar : public QToolBar {
    Q_OBJECT
public:
    Toolbar(App& app, QWidget* parent = nullptr);
    void refresh();

private:
    void rebuild(const ToolbarModel& m);
    void apply(const ToolbarModel& m);
    QAction* button(const QString& text, UiAction a, int arg = 0);
    void label(const QString& text, bool dim = false);

    App& app_;
    bool built_ = false;
    ToolbarModel last_;
    // Per-frame updates for the current variant's widgets.
    std::vector<std::function<void(const ToolbarModel&)>> sync_;
};

} // namespace shitcad
