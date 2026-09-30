#pragma once
#include "App.h"

#include <QDialog>

#undef near
#undef far

class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;

namespace shitcad {

// Qt front end for MeshImportModel:
// STL has no units, so this asks for one while showing the resulting size.
class MeshImportDialog : public QDialog {
    Q_OBJECT
public:
    MeshImportDialog(App& app, QWidget* parent = nullptr);
    void refresh();

protected:
    // Escape and the window's X both land here; App is told, or the next
    // refresh would show the dialog again.
    void reject() override;

private:
    App& app_;
    std::string shownPath_;            // reloads the fields when a new file arrives
    QLabel* path_ = nullptr;
    QLabel* error_ = nullptr;
    QLabel* triangles_ = nullptr;
    QLineEdit* name_ = nullptr;
    QComboBox* unit_ = nullptr;
    QLabel* size_ = nullptr;
    QLabel* warning_ = nullptr;
    QLabel* blocked_ = nullptr;        // why Import is disabled
    QPushButton* import_ = nullptr;
    QWidget* form_ = nullptr;          // everything hidden when the file failed to read
};

// Qt front end for CadImportModel: what a STEP / IGES file holds, and which
// way is up in it.
class CadImportDialog : public QDialog {
    Q_OBJECT
public:
    CadImportDialog(App& app, QWidget* parent = nullptr);
    void refresh();

protected:
    void reject() override;            // as MeshImportDialog

private:
    App& app_;
    std::string shownPath_;
    QLabel* path_ = nullptr;
    QLabel* error_ = nullptr;
    QLabel* contents_ = nullptr;       // format, unit, solid / surface counts
    QLabel* size_ = nullptr;
    QLabel* warning_ = nullptr;        // curves that make no body
    QLabel* blocked_ = nullptr;        // why Import is disabled
    QPushButton* import_ = nullptr;
    QLineEdit* name_ = nullptr;
    QComboBox* up_ = nullptr;
    QWidget* form_ = nullptr;
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
