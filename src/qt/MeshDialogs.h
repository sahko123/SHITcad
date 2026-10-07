#pragma once
#include "App.h"

#include <QDialog>

#include <functional>
#include <string>

#undef near
#undef far

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QFormLayout;
class QLabel;
class QLineEdit;
class QPushButton;

namespace shitcad {

// What the STL and STEP / IGES import dialogs share: the file path, a read error (which hides the
// form), a form each fills with its own rows, the notice of why Import is unavailable, and the
// Import / Cancel buttons.
class ImportDialogBase : public QDialog {
public:
    using Action = std::function<void(App&)>;

protected:
    ImportDialogBase(App& app, Action confirm, Action cancel, QWidget* parent);

    // Escape and the window's X both land here; App is told, or the next
    // refresh would show the dialog again.
    void reject() override;

    // The first part of refresh(): show or hide the dialog and fill in the path, error and
    // blocked notice. False when there is nothing more to show (closed, or the file failed).
    bool refreshHead(bool open, const std::string& path, const std::string& error, const std::string& blocked);
    // True the first time a new file is shown: the fields then take App's defaults.
    bool isNewFile(const std::string& path);

    App& app_;
    Action cancel_;
    std::string shownPath_;
    QLabel* path_ = nullptr;
    QLabel* error_ = nullptr;
    QLabel* size_ = nullptr;
    QLabel* warning_ = nullptr;
    QLabel* blocked_ = nullptr;        // why Import is disabled
    QLineEdit* name_ = nullptr;
    QPushButton* import_ = nullptr;
    QWidget* form_ = nullptr;          // everything hidden when the file failed to read
    QFormLayout* formLayout_ = nullptr; // rows added by the derived dialog
};

// Qt front end for MeshImportModel:
// STL has no units, so this asks for one while showing the resulting size.
class MeshImportDialog : public ImportDialogBase {
    Q_OBJECT
public:
    MeshImportDialog(App& app, QWidget* parent = nullptr);
    void refresh();

private:
    QLabel* triangles_ = nullptr;
    QComboBox* unit_ = nullptr;
    QCheckBox* keepInside_ = nullptr;  // shown for a solid-wall export
};

// Qt front end for CadImportModel: what a STEP / IGES file holds, and which
// way is up in it.
class CadImportDialog : public ImportDialogBase {
    Q_OBJECT
public:
    CadImportDialog(App& app, QWidget* parent = nullptr);
    void refresh();

private:
    QLabel* contents_ = nullptr;       // format, unit, solid / surface counts
    QComboBox* up_ = nullptr;
};

// Qt front end for MeshPlaceModel:
// rotate and move an import (STL, STEP or IGES) into place, applied live.
class MeshPlacePanel : public QDialog {
    Q_OBJECT
public:
    MeshPlacePanel(App& app, QWidget* parent = nullptr);
    void refresh();

protected:
    // Escape and the window's X both land here; App is told, or the next
    // refresh would show the dialog again.
    void reject() override;

private:
    void pushPosition();

    App& app_;
    QLabel* error_ = nullptr;
    QWidget* form_ = nullptr;
    QComboBox* unit_ = nullptr;
    QWidget* unitRow_ = nullptr;       // hidden for STEP / IGES, which carry their unit
    QLabel* size_ = nullptr;
    QLabel* bottom_ = nullptr;
    QDoubleSpinBox* angle_ = nullptr;
    QComboBox* angleAxis_ = nullptr;
    QDoubleSpinBox* pos_[3] = {};
    bool closing_ = false;             // Done / Cancel already handled the close
};

} // namespace shitcad
