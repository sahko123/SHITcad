// SHITcad: a QMainWindow whose central widget is the 3D viewport, with the
// panels as Qt widgets around and over it. Each panel reads its model from
// App after every frame (ViewportWidget::frameBuilt) and changes App only
// through App::post.
#include "CrashLogger.h"
#include "ViewportWidget.h"
#include "Toolbar.h"
#include "PreferencesDialog.h"
#include "MeshDialogs.h"
#include "PlaneDialogs.h"
#include "ObjectTree.h"
#include "ToolPanel.h"
#include "SectionControls.h"
#include "Timeline.h"
#include "SimulationPanel.h"
#include "InViewport.h"
#include "KeyRouting.h"

#include <QApplication>
#include <QMainWindow>
#include <QPalette>
#include <QSettings>
#include <QStyle>
#include <QStyleFactory>
#include <QSurfaceFormat>

namespace {

// Fusion with a light or dark palette, following the Light Mode preference
// (the 3D view's colours follow it through Preferences::applyTheme).
void applyUiTheme(bool light) {
    QApplication::setStyle(QStyleFactory::create("Fusion"));
    if (light) {
        QApplication::setPalette(QApplication::style()->standardPalette());
        return;
    }
    QPalette p;
    const QColor window(45, 45, 50), base(32, 32, 36), text(230, 230, 235), mid(70, 70, 78);
    const QColor accent(66, 120, 200), disabled(125, 125, 132);
    p.setColor(QPalette::Window, window);
    p.setColor(QPalette::WindowText, text);
    p.setColor(QPalette::Base, base);
    p.setColor(QPalette::AlternateBase, window);
    p.setColor(QPalette::ToolTipBase, base);
    p.setColor(QPalette::ToolTipText, text);
    p.setColor(QPalette::Text, text);
    p.setColor(QPalette::Button, window);
    p.setColor(QPalette::ButtonText, text);
    p.setColor(QPalette::BrightText, Qt::white);
    p.setColor(QPalette::Mid, mid);
    p.setColor(QPalette::Dark, base);
    p.setColor(QPalette::Light, mid);
    p.setColor(QPalette::Highlight, accent);
    p.setColor(QPalette::HighlightedText, Qt::white);
    p.setColor(QPalette::Link, accent);
    p.setColor(QPalette::PlaceholderText, disabled);
    for (auto role : {QPalette::WindowText, QPalette::Text, QPalette::ButtonText})
        p.setColor(QPalette::Disabled, role, disabled);
    QApplication::setPalette(p);
}

} // namespace

int main(int argc, char** argv) {
    shitcad::initCrashLogger();

    // Must be set before the QApplication exists. Stencil is needed by the
    // section-view cap; the renderer is OpenGL 3.3 core.
    QSurfaceFormat fmt;
    fmt.setRenderableType(QSurfaceFormat::OpenGL);
    fmt.setVersion(3, 3);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    fmt.setDepthBufferSize(24);
    fmt.setStencilBufferSize(8);
    fmt.setSwapInterval(1);
    QSurfaceFormat::setDefaultFormat(fmt);

    QApplication qapp(argc, argv);
    QApplication::setApplicationName("SHITcad");
    applyUiTheme(true);

    QMainWindow window;
    window.setWindowTitle("SHITcad");
    auto* viewport = new shitcad::ViewportWidget(&window);
    window.setCentralWidget(viewport);

    auto* toolbar = new shitcad::Toolbar(viewport->app(), &window);
    toolbar->setObjectName("toolbar");   // saveState() keys toolbars by name
    window.addToolBar(Qt::TopToolBarArea, toolbar);
    QObject::connect(viewport, &shitcad::ViewportWidget::frameBuilt, toolbar, &shitcad::Toolbar::refresh);

    auto* prefs = new shitcad::PreferencesDialog(viewport->app(), &window);
    QObject::connect(viewport, &shitcad::ViewportWidget::frameBuilt, prefs, &shitcad::PreferencesDialog::refresh);

    auto* meshImport = new shitcad::MeshImportDialog(viewport->app(), &window);
    QObject::connect(viewport, &shitcad::ViewportWidget::frameBuilt, meshImport, &shitcad::MeshImportDialog::refresh);

    auto* meshPlace = new shitcad::MeshPlacePanel(viewport->app(), &window);
    QObject::connect(viewport, &shitcad::ViewportWidget::frameBuilt, meshPlace, &shitcad::MeshPlacePanel::refresh);

    auto* addPlane = new shitcad::AddPlaneDialog(viewport->app(), &window);
    QObject::connect(viewport, &shitcad::ViewportWidget::frameBuilt, addPlane, &shitcad::AddPlaneDialog::refresh);

    auto* tangentPlane = new shitcad::TangentPlaneDialog(viewport->app(), &window);
    QObject::connect(viewport, &shitcad::ViewportWidget::frameBuilt, tangentPlane, &shitcad::TangentPlaneDialog::refresh);

    auto* tree = new shitcad::ObjectTree(viewport->app(), &window);
    window.addDockWidget(Qt::LeftDockWidgetArea, tree);
    window.resizeDocks({tree}, {200}, Qt::Horizontal);
    QObject::connect(viewport, &shitcad::ViewportWidget::frameBuilt, tree, &shitcad::ObjectTree::refresh);

    auto* toolPanel = new shitcad::ToolPanel(viewport->app(), viewport);
    QObject::connect(viewport, &shitcad::ViewportWidget::frameBuilt, toolPanel, &shitcad::ToolPanel::refresh);

    auto* section = new shitcad::SectionWindow(viewport->app(), &window);
    QObject::connect(viewport, &shitcad::ViewportWidget::frameBuilt, section, &shitcad::SectionWindow::refresh);

    auto* timeline = new shitcad::Timeline(viewport->app(), viewport);
    QObject::connect(viewport, &shitcad::ViewportWidget::frameBuilt, timeline, &shitcad::Timeline::refresh);

    auto* simulation = new shitcad::SimulationPanel(viewport->app(), &window);
    window.addDockWidget(Qt::RightDockWidgetArea, simulation);
    QObject::connect(viewport, &shitcad::ViewportWidget::frameBuilt, simulation, &shitcad::SimulationPanel::refresh);

    auto* inlineInput = new shitcad::InlineInput(viewport->app(), viewport);
    auto* dimension = new shitcad::DimensionPanel(viewport->app(), viewport);
    QObject::connect(viewport, &shitcad::ViewportWidget::frameBuilt, inlineInput, &shitcad::InlineInput::refresh);
    QObject::connect(viewport, &shitcad::ViewportWidget::frameBuilt, dimension, &shitcad::DimensionPanel::refresh);

    // The widgets' theme follows the Light Mode preference.
    QObject::connect(viewport, &shitcad::ViewportWidget::frameBuilt, &window, [viewport] {
        static int applied = 1;   // light, set above
        const int light = viewport->app().preferences().lightMode ? 1 : 0;
        if (light != applied) { applied = light; applyUiTheme(light != 0); }
    });

    // Docks can take the keyboard; shortcuts still reach the viewport.
    qapp.installEventFilter(new shitcad::KeyRouting(&window, viewport));

    // Window geometry and dock layout persist between runs. What is open
    // (the tree's T toggle) is App state and is re-applied every frame.
    QSettings settings("SHITcad", "SHITcad");
    window.resize(1280, 720);
    window.restoreState(settings.value("window/state").toByteArray());
    if (window.restoreGeometry(settings.value("window/geometry").toByteArray()))
        window.show();
    else
        window.showMaximized();
    QObject::connect(&qapp, &QApplication::aboutToQuit, &window, [&window] {
        QSettings s("SHITcad", "SHITcad");
        s.setValue("window/geometry", window.saveGeometry());
        s.setValue("window/state", window.saveState());
    });
    viewport->setFocus();

    return qapp.exec();
}
