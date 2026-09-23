#pragma once
#include "App.h"

#include <QDialog>

#undef near
#undef far

class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;

namespace shitcad {

// Qt front end for MeshImportModel (the ImGui one is App::drawMeshImportDialog):
// STL has no units, so this asks for one while showing the resulting size.
class MeshImportDialog : public QDialog {
    Q_OBJECT
public:
    MeshImportDialog(App& app, QWidget* parent = nullptr);
    void refresh();

protected:
    void closeEvent(QCloseEvent* e) override;

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
    QWidget* form_ = nullptr;          // everything hidden when the file failed to read
};

// Qt front end for MeshPlaceModel (the ImGui one is App::drawMeshPlacePanel):
// rotate and move an imported mesh into place, applied live.
class MeshPlacePanel : public QDialog {
    Q_OBJECT
public:
    MeshPlacePanel(App& app, QWidget* parent = nullptr);
    void refresh();

protected:
    void closeEvent(QCloseEvent* e) override;

private:
    void pushPosition();

    App& app_;
    QLabel* error_ = nullptr;
    QWidget* form_ = nullptr;
    QComboBox* unit_ = nullptr;
    QLabel* size_ = nullptr;
    QLabel* bottom_ = nullptr;
    QDoubleSpinBox* angle_ = nullptr;
    QComboBox* angleAxis_ = nullptr;
    QDoubleSpinBox* pos_[3] = {};
    bool closing_ = false;             // Done / Cancel already handled the close
};

} // namespace shitcad
