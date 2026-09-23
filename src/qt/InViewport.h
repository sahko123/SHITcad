#pragma once
#include "App.h"

#include <QFrame>
#include <QKeyEvent>
#include <QLineEdit>

#undef near
#undef far

class QCheckBox;
class QLabel;
class QPushButton;

namespace shitcad {

// A value field inside the viewport: Enter and Escape are its own (they
// apply or cancel), and no key it gets travels on to the viewport.
class ViewportField : public QLineEdit {
    Q_OBJECT
public:
    using QLineEdit::QLineEdit;
signals:
    void submitted();
    void cancelled();
protected:
    void keyPressEvent(QKeyEvent* e) override;
    void keyReleaseEvent(QKeyEvent* e) override;
};

// Qt front end for InlineInputModel (the ImGui one is App::drawInlineDimInput):
// the value box beside the cursor while drawing a circle or a fillet.
class InlineInput : public ViewportField {
    Q_OBJECT
public:
    InlineInput(App& app, QWidget* viewport);
    void refresh();

private:
    App& app_;
    QWidget* viewport_;
    std::string lastText_;   // the model's text when last seen
};

// Qt front end for DimensionPanelModel (the ImGui one is App::drawDimensionPanel):
// floats over the viewport's top-right corner, like the tool panel. The value
// field takes the keyboard back whenever the model asks, as the ImGui field
// does every frame while a label is being placed.
class DimensionPanel : public QFrame {
    Q_OBJECT
public:
    DimensionPanel(App& app, QWidget* viewport);
    void refresh();

protected:
    void keyPressEvent(QKeyEvent* e) override { e->accept(); }
    void keyReleaseEvent(QKeyEvent* e) override { e->accept(); }

private:
    App& app_;
    QWidget* viewport_;
    QLabel* warning_ = nullptr;
    QLabel* hint_ = nullptr;
    QLabel* kind_ = nullptr;
    QLabel* placeHint_ = nullptr;
    ViewportField* value_ = nullptr;
    QCheckBox* driven_ = nullptr;
    QPushButton* apply_ = nullptr;
    QWidget* editing_ = nullptr;
    std::string lastText_;
};

} // namespace shitcad
