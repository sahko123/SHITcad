#pragma once
#include "App.h"

#include <QDialog>

#undef near
#undef far

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;

namespace shitcad {

// Qt front end for Preferences.
// Non-modal; shown while App::preferencesOpen(). Edits go to App through
// post(setPreferences); refresh() loads App's values back when they differ
// from what the dialog last sent (light mode resets the colours, for one).
class PreferencesDialog : public QDialog {
    Q_OBJECT
public:
    PreferencesDialog(App& app, QWidget* parent = nullptr);
    void refresh();

protected:
    // Escape and the window's X both land here; App is told, or the next
    // refresh would show the dialog again.
    void reject() override;

private:
    void load(const Preferences& p);   // App -> widgets, without echoing back
    void push();                       // widgets -> App
    QPushButton* colourButton(float* rgba, bool alpha, const QString& title);

    App& app_;
    Preferences shown_;                // what the widgets show / last sent
    bool loaded_ = false;

    QCheckBox* lightMode_ = nullptr;
    QCheckBox* showEdges_ = nullptr;
    QDoubleSpinBox* lineWidth_ = nullptr;
    QDoubleSpinBox* edgeWidth_ = nullptr;
    QDoubleSpinBox* tangentSnap_ = nullptr;
    QComboBox* backend_ = nullptr;
    QLabel* backendNote_ = nullptr;
    struct Swatch { QPushButton* button; float* value; bool alpha; };
    std::vector<Swatch> swatches_;     // value points into shown_
};

} // namespace shitcad
