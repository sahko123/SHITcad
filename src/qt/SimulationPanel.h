#pragma once
#include "App.h"

#include <QDockWidget>

#undef near
#undef far

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QPlainTextEdit;
class QPushButton;
class QSlider;
class QSpinBox;
class QTableWidget;
class QVBoxLayout;

namespace shitcad {

class SectionControls;
class ResultLegend;

// Qt front end for the Simulation panel models (SimSetupModel, SimRunModel,
// SimResultsModel). A dock on the right, shown in the Simulation workspace.
// Live edits (sliders) change the set-up as they move and commit one undo
// step on release; typed values apply and commit when the field is finished.
class SimulationPanel : public QDockWidget {
    Q_OBJECT
public:
    SimulationPanel(App& app, QWidget* parent = nullptr);
    void refresh();

private:
    QWidget* buildSurfaces();
    QWidget* buildNozzles();
    QWidget* buildRunSettings();
    QWidget* buildRun();
    QWidget* buildResults();
    void refreshSetup(const App::SimSetupModel& m);
    void addSkinRows(QVBoxLayout* rc, QWidget* row, const App::SimSetupModel::Surface& s);
    void refreshRun(const App::SimRunModel& m);
    void refreshResults(const App::SimResultsModel& m);
    void pushPosition();
    void pushAxis();

    App& app_;
    QVBoxLayout* surfaceRows_ = nullptr;
    QLabel* surfaceNote_ = nullptr;
    std::string shownSurfaces_;
    std::vector<QComboBox*> roleCombos_;

    QPushButton* place_ = nullptr;
    QDoubleSpinBox* standoff_ = nullptr;
    QListWidget* nozzles_ = nullptr;
    std::string shownNozzles_;
    QWidget* detail_ = nullptr;
    uint32_t detailId_ = 0;
    QLineEdit* name_ = nullptr;
    QWidget* placement_ = nullptr;
    QDoubleSpinBox* pos_[3] = {};
    QDoubleSpinBox* axis_[3] = {};
    QLabel* orphan_ = nullptr;
    QSlider* halfAngle_ = nullptr;
    QLabel* halfAngleText_ = nullptr;
    QDoubleSpinBox* flow_ = nullptr;
    QDoubleSpinBox* pressure_ = nullptr;

    QSpinBox* rays_ = nullptr;
    QSpinBox* bounces_ = nullptr;
    QWidget* viewBody_ = nullptr;
    SectionControls* section_ = nullptr;

    QLineEdit* cipSim_ = nullptr;
    QLineEdit* python_ = nullptr;
    QLabel* engineNote_ = nullptr;
    QLabel* runStatus_ = nullptr;
    QPushButton* runButton_ = nullptr;
    QPushButton* cancelButton_ = nullptr;
    QLabel* runError_ = nullptr;
    QPlainTextEdit* log_ = nullptr;
    size_t shownLog_ = 0;

    QWidget* results_ = nullptr;
    QLabel* stale_ = nullptr;
    QCheckBox* showResults_ = nullptr;
    QComboBox* field_ = nullptr;
    std::vector<std::string> shownFields_;
    ResultLegend* legend_ = nullptr;
    QTableWidget* coverage_ = nullptr;
    std::string shownCoverage_;
    QLabel* paraview_ = nullptr;

    QLabel* message_ = nullptr;
};

} // namespace shitcad
