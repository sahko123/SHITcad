#pragma once
#include "App.h"

#include <QFrame>

#undef near
#undef far

class QCheckBox;
class QComboBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QStackedWidget;
class QVBoxLayout;

namespace shitcad {

// Qt front end for the Extrude, Revolve, Loft and Boolean panels (the ImGui
// ones are App::draw*Panel). One frame over the top-right corner of the
// viewport, where the ImGui panels sit, showing the page for the active tool.
// It floats rather than docks so that starting a tool does not resize the 3D
// view. Keys it does not use are swallowed here: those meant for the viewport
// reach it through KeyRouting, and an Escape that only leaves a text field
// must not also cancel the tool.
class ToolPanel : public QFrame {
    Q_OBJECT
public:
    ToolPanel(App& app, QWidget* viewport);
    void refresh();

protected:
    void keyPressEvent(QKeyEvent* e) override;
    void keyReleaseEvent(QKeyEvent* e) override;

private:
    enum Page { Extrude, Revolve, Loft, Boolean, None };
    QWidget* buildExtrude();
    QWidget* buildRevolve();
    QWidget* buildLoft();
    QWidget* buildBoolean();
    void refreshExtrude(const App::ExtrudePanelModel& m);
    void refreshRevolve(const App::RevolvePanelModel& m);
    void refreshLoft(const App::LoftPanelModel& m);
    void refreshBoolean(const App::BooleanPanelModel& m);
    // A typed value is applied on Enter or focus loss; Enter also hands the
    // keyboard back to the viewport, as an ImGui field lets go on Enter.
    QLineEdit* valueField(QWidget* parent, void (App::*apply)(const std::string&));
    void setField(QLineEdit* field, const std::string& text);

    App& app_;
    QWidget* viewport_;
    QLabel* title_ = nullptr;
    QStackedWidget* pages_ = nullptr;
    Page shown_ = None;

    QComboBox* exOp_ = nullptr;
    QLineEdit* exDist_ = nullptr;
    QComboBox* exDir_ = nullptr;
    QLineEdit* exOffset_ = nullptr;
    QLabel* exProfiles_ = nullptr;
    QPushButton* exOk_ = nullptr;

    QComboBox* rvOp_ = nullptr;
    QLineEdit* rvAngle_ = nullptr;
    QLabel* rvAxis_ = nullptr;
    QPushButton* rvChangeAxis_ = nullptr;
    QLabel* rvPickHint_ = nullptr;
    QPushButton* rvSelectAxis_ = nullptr;
    QLabel* rvProfiles_ = nullptr;
    QPushButton* rvSelectProfiles_ = nullptr;
    QPushButton* rvOk_ = nullptr;

    QWidget* lfSections_ = nullptr;
    QWidget* lfCandidates_ = nullptr;
    QWidget* lfProfiles_ = nullptr;
    QLabel* lfProfilesTitle_ = nullptr;
    QCheckBox* lfSolid_ = nullptr;
    QPushButton* lfOk_ = nullptr;
    std::vector<int> shownSections_;        // plane, profile, count per section, flattened
    std::vector<std::string> shownCandidates_;
    std::vector<int> shownProfileCounts_;

    QLabel* blHint_ = nullptr;
    QLabel* blTarget_ = nullptr;
    QPushButton* blClearTarget_ = nullptr;
    QLabel* blTool_ = nullptr;
    QPushButton* blClearTool_ = nullptr;
    QLabel* blToolHint_ = nullptr;
    QPushButton* blApply_ = nullptr;
    QLabel* blPreview_ = nullptr;
};

} // namespace shitcad
