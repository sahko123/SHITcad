#include "KeyRouting.h"
#include "ViewportWidget.h"

#include <QAbstractSpinBox>
#include <QApplication>
#include <QComboBox>
#include <QDialog>
#include <QDockWidget>
#include <QKeyEvent>
#include <QLineEdit>
#include <QMainWindow>
#include <QPlainTextEdit>
#include <QTextEdit>

namespace shitcad {

KeyRouting::KeyRouting(QMainWindow* window, ViewportWidget* viewport)
    : QObject(window), window_(window), viewport_(viewport) {}

static bool isTextInput(const QWidget* w) {
    if (!w) return false;
    if (qobject_cast<const QLineEdit*>(w) || qobject_cast<const QAbstractSpinBox*>(w) ||
        qobject_cast<const QTextEdit*>(w) || qobject_cast<const QPlainTextEdit*>(w))
        return true;
    const auto* combo = qobject_cast<const QComboBox*>(w);
    return combo && combo->isEditable();
}

static bool isNavigationKey(int key) {
    switch (key) {
    case Qt::Key_Up: case Qt::Key_Down: case Qt::Key_Left: case Qt::Key_Right:
    case Qt::Key_Home: case Qt::Key_End: case Qt::Key_PageUp: case Qt::Key_PageDown:
    case Qt::Key_Tab: case Qt::Key_Backtab: case Qt::Key_Space:
        return true;
    default:
        return false;
    }
}

bool KeyRouting::eventFilter(QObject* obj, QEvent* e) {
    if (e->type() != QEvent::KeyPress && e->type() != QEvent::KeyRelease) return false;
    auto* w = qobject_cast<QWidget*>(obj);
    if (!w) return false;
    auto* ke = static_cast<QKeyEvent*>(e);
    const quint32 scan = ke->nativeScanCode();

    // Widgets inside the viewport (the tool panel) are routed like the docks.
    if (w == viewport_) {
        if (e->type() == QEvent::KeyRelease && !ke->isAutoRepeat()) held_.remove(scan);
        return false;
    }
    if (e->type() == QEvent::KeyRelease && !ke->isAutoRepeat() && held_.remove(scan)) {
        QCoreApplication::sendEvent(viewport_, e);
        return true;
    }
    if (QApplication::activeModalWidget()) return false;

    // The main window, its docks (floating ones included) and the tool
    // windows it owns. Escape and Enter in a tool window stay with it: they
    // close it and press its default button.
    QWidget* top = w->window();
    if (top != window_) {
        if (top->parentWidget() != window_) return false;
        if (qobject_cast<QDialog*>(top)) {
            const int k = ke->key();
            if (k == Qt::Key_Escape || k == Qt::Key_Return || k == Qt::Key_Enter) return false;
        } else if (!qobject_cast<QDockWidget*>(top)) {
            return false;
        }
    }

    // Route an event once, where it is first delivered (to the focus widget).
    // An event a widget ignored then travels up its parents, and the focus
    // may have moved meanwhile: QLineEdit applies a value on Return, then
    // ignores the key, and that Return must not also commit the tool.
    QWidget* focus = QApplication::focusWidget();
    if (focus && w != focus) return false;
    if (isTextInput(focus)) return false;
    if (isNavigationKey(ke->key()) && !(ke->modifiers() & Qt::ControlModifier)) return false;

    if (e->type() == QEvent::KeyPress && !ke->isAutoRepeat() && scan) held_.insert(scan);
    QCoreApplication::sendEvent(viewport_, e);
    return true;
}

} // namespace shitcad
