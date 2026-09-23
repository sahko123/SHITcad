#pragma once
#include <QObject>
#include <QSet>

class QMainWindow;

namespace shitcad {

class ViewportWidget;

// Keeps the keyboard shortcuts working when a dock has focus. Every shortcut
// is handled by the viewport's input handlers, which already know the mode
// (Ctrl+Z undoes the sketch in a sketch and the history elsewhere, E enters
// extrude but Ctrl+E exports). Installed on the application, this forwards key
// events from the main window, its docks, the widgets over the viewport and
// the tool windows the main window owns to the viewport, except:
//   - while a text field has focus,
//   - navigation keys the focused widget uses itself (arrows, Home/End,
//     Page Up/Down, Tab, Space),
//   - Escape and Enter in a tool window, which close it or press its button,
//   - anything while a modal dialog is open, or in other top-level windows.
// The release of a forwarded press is always forwarded, wherever the focus
// has moved since, or the viewport would see the key held and repeat it.
class KeyRouting : public QObject {
public:
    KeyRouting(QMainWindow* window, ViewportWidget* viewport);

protected:
    bool eventFilter(QObject* obj, QEvent* e) override;

private:
    QMainWindow* window_;
    ViewportWidget* viewport_;
    QSet<quint32> held_;   // scan codes of forwarded presses not yet released
};

} // namespace shitcad
