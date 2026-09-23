// Qt host (SHITcadQt): a QMainWindow whose central widget is the viewport.
// The whole ImGui UI still runs inside the viewport; panels move to Qt
// widgets one at a time (docs/qt-migration-plan.md, Phase 5).
#include "CrashLogger.h"
#include "ViewportWidget.h"
#include "Toolbar.h"
#include "PreferencesDialog.h"
#include "MeshDialogs.h"

#include <QApplication>
#include <QMainWindow>
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
    window.resize(1280, 720);
    window.showMaximized();
    viewport->setFocus();

    return qapp.exec();
}
