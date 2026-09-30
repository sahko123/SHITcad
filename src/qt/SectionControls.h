#pragma once
#include "App.h"

#include <QDialog>

#undef near
#undef far

class QCheckBox;
class QLabel;
class QPushButton;
class QRadioButton;
class QSlider;

namespace shitcad {

// Qt front end for SectionModel.
// A widget of its own so the Simulation panel can hold it too (Phase 5.9).
class SectionControls : public QWidget {
    Q_OBJECT
public:
    SectionControls(App& app, QWidget* parent = nullptr);
    void refresh(const App::SectionModel& m);

private:
    float sliderMm(int value) const;

    App& app_;
    QCheckBox* enabled_ = nullptr;
    QLabel* offHint_ = nullptr;
    QWidget* body_ = nullptr;
    QRadioButton* axis_[3] = {};
    QCheckBox* flip_ = nullptr;
    QSlider* slider_ = nullptr;
    QLabel* position_ = nullptr;
    QCheckBox* cap_ = nullptr;
    QLabel* openNote_ = nullptr;
    QLabel* watertightNote_ = nullptr;
    float lo_ = 0.0f, hi_ = 0.0f;   // the slider's range, from the last refresh
};

// The Model workspace's Section window (the toolbar's Section button).
class SectionWindow : public QDialog {
    Q_OBJECT
public:
    SectionWindow(App& app, QWidget* parent = nullptr);
    void refresh();

protected:
    void reject() override;   // Escape or the window's X

private:
    App& app_;
    SectionControls* controls_ = nullptr;
    bool placed_ = false;
};

} // namespace shitcad
