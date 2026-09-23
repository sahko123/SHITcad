// Qt host (SHITcadQt): a QMainWindow whose central widget is the viewport.
// The whole ImGui UI still runs inside the viewport; panels move to Qt
// widgets one at a time (docs/qt-migration-plan.md, Phase 5).
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
#include <QSettings>
#include <QSurfaceFormat>

int main(int argc, char** argv) {
    shitcad::initCrashLogger();

    // Must be set before the QApplication exists. Stencil is needed by the
    // section-view cap; the context matches the GLFW host's (3.3 core).
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

    QMainWindow window;
    window.setWindowTitle("SHITcad");
    auto* viewport = new shitcad::ViewportWidget(&window);
    window.setCentralWidget(viewport);

    // Panels that have moved to Qt; App skips their ImGui versions.
    viewport->app().setHostPanel(shitcad::App::HostToolbar);
    auto* toolbar = new shitcad::Toolbar(viewport->app(), &window);
    toolbar->setObjectName("toolbar");   // saveState() keys toolbars by name
    window.addToolBar(Qt::TopToolBarArea, toolbar);
    QObject::connect(viewport, &shitcad::ViewportWidget::frameBuilt, toolbar, &shitcad::Toolbar::refresh);

    viewport->app().setHostPanel(shitcad::App::HostPreferences);
    auto* prefs = new shitcad::PreferencesDialog(viewport->app(), &window);
    QObject::connect(viewport, &shitcad::ViewportWidget::frameBuilt, prefs, &shitcad::PreferencesDialog::refresh);

    viewport->app().setHostPanel(shitcad::App::HostMeshImport);
    auto* meshImport = new shitcad::MeshImportDialog(viewport->app(), &window);
    QObject::connect(viewport, &shitcad::ViewportWidget::frameBuilt, meshImport, &shitcad::MeshImportDialog::refresh);

    viewport->app().setHostPanel(shitcad::App::HostMeshPlace);
    auto* meshPlace = new shitcad::MeshPlacePanel(viewport->app(), &window);
    QObject::connect(viewport, &shitcad::ViewportWidget::frameBuilt, meshPlace, &shitcad::MeshPlacePanel::refresh);

    viewport->app().setHostPanel(shitcad::App::HostAddPlane);
    auto* addPlane = new shitcad::AddPlaneDialog(viewport->app(), &window);
    QObject::connect(viewport, &shitcad::ViewportWidget::frameBuilt, addPlane, &shitcad::AddPlaneDialog::refresh);

    viewport->app().setHostPanel(shitcad::App::HostTangentPlane);
    auto* tangentPlane = new shitcad::TangentPlaneDialog(viewport->app(), &window);
    QObject::connect(viewport, &shitcad::ViewportWidget::frameBuilt, tangentPlane, &shitcad::TangentPlaneDialog::refresh);

    viewport->app().setHostPanel(shitcad::App::HostObjectTree);
    auto* tree = new shitcad::ObjectTree(viewport->app(), &window);
    window.addDockWidget(Qt::LeftDockWidgetArea, tree);
    window.resizeDocks({tree}, {200}, Qt::Horizontal);
    QObject::connect(viewport, &shitcad::ViewportWidget::frameBuilt, tree, &shitcad::ObjectTree::refresh);

    viewport->app().setHostPanel(shitcad::App::HostToolPanels);
    auto* toolPanel = new shitcad::ToolPanel(viewport->app(), viewport);
    QObject::connect(viewport, &shitcad::ViewportWidget::frameBuilt, toolPanel, &shitcad::ToolPanel::refresh);

    viewport->app().setHostPanel(shitcad::App::HostSection);
    auto* section = new shitcad::SectionWindow(viewport->app(), &window);
    QObject::connect(viewport, &shitcad::ViewportWidget::frameBuilt, section, &shitcad::SectionWindow::refresh);

    viewport->app().setHostPanel(shitcad::App::HostTimeline);
    auto* timeline = new shitcad::Timeline(viewport->app(), viewport);
    QObject::connect(viewport, &shitcad::ViewportWidget::frameBuilt, timeline, &shitcad::Timeline::refresh);

    viewport->app().setHostPanel(shitcad::App::HostSimulation);
    auto* simulation = new shitcad::SimulationPanel(viewport->app(), &window);
    window.addDockWidget(Qt::RightDockWidgetArea, simulation);
    QObject::connect(viewport, &shitcad::ViewportWidget::frameBuilt, simulation, &shitcad::SimulationPanel::refresh);

    viewport->app().setHostPanel(shitcad::App::HostInViewport);
    auto* inlineInput = new shitcad::InlineInput(viewport->app(), viewport);
    auto* dimension = new shitcad::DimensionPanel(viewport->app(), viewport);
    QObject::connect(viewport, &shitcad::ViewportWidget::frameBuilt, inlineInput, &shitcad::InlineInput::refresh);
    QObject::connect(viewport, &shitcad::ViewportWidget::frameBuilt, dimension, &shitcad::DimensionPanel::refresh);

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
