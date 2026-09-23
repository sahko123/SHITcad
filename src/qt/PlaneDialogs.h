#pragma once
#include "App.h"

#include <QDialog>

#undef near
#undef far

class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QRadioButton;
class QSlider;
class QVBoxLayout;

namespace shitcad {

// Qt front end for AddPlaneModel (the ImGui one is App::drawAddPlaneDialog):
// a reference plane offset from another plane, or from a face picked in the
// viewport.
class AddPlaneDialog : public QDialog {
    Q_OBJECT
public:
    AddPlaneDialog(App& app, QWidget* parent = nullptr);
    void refresh();

protected:
    void closeEvent(QCloseEvent* e) override;

private:
    void rebuildSources(const App::AddPlaneModel& m);

    App& app_;
    QRadioButton* fromPlane_ = nullptr;
    QRadioButton* fromFace_ = nullptr;
    QWidget* sourceList_ = nullptr;
    QVBoxLayout* sourceLayout_ = nullptr;
    std::vector<QRadioButton*> sources_;   // parallel to the model's sources
    QLabel* faceState_ = nullptr;
    QLineEdit* offset_ = nullptr;
    QLineEdit* name_ = nullptr;
    QPushButton* create_ = nullptr;
    size_t shownSources_ = 0;
};

// Qt front end for TangentPlaneModel (the ImGui one is
// App::drawTangentPlaneDialog): a plane tangent to a cylinder, at an angle
// from the clicked point. Create also starts a sketch on it.
class TangentPlaneDialog : public QDialog {
    Q_OBJECT
public:
    TangentPlaneDialog(App& app, QWidget* parent = nullptr);
    void refresh();

protected:
    void closeEvent(QCloseEvent* e) override;

private:
    App& app_;
    QLineEdit* name_ = nullptr;
    QDoubleSpinBox* angle_ = nullptr;
    QSlider* slider_ = nullptr;
};

} // namespace shitcad
