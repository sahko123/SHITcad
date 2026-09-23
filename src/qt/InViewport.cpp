#include "InViewport.h"

#include <QCheckBox>
#include <QCursor>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include <algorithm>

namespace shitcad {

void ViewportField::keyPressEvent(QKeyEvent* e) {
    switch (e->key()) {
    case Qt::Key_Return:
    case Qt::Key_Enter:
        emit submitted();
        break;
    case Qt::Key_Escape:
        emit cancelled();
        break;
    default:
        QLineEdit::keyPressEvent(e);
        break;
    }
    e->accept();
}

void ViewportField::keyReleaseEvent(QKeyEvent* e) {
    QLineEdit::keyReleaseEvent(e);
    e->accept();
}

// ---- inline value box ----------------------------------------------------

InlineInput::InlineInput(App& app, QWidget* viewport) : ViewportField(viewport), app_(app), viewport_(viewport) {
    setFixedWidth(100);
    hide();
    App* a = &app_;
    connect(this, &QLineEdit::textEdited, this, [this, a](const QString& t) {
        lastText_ = t.toUtf8().toStdString();
        const std::string s = lastText_;
        a->post([a, s] { a->setInlineInputText(s); });
    });
    connect(this, &ViewportField::submitted, this, [a] { a->post([a] { a->submitInlineInput(); }); });
    connect(this, &ViewportField::cancelled, this, [a] { a->post([a] { a->cancelInlineInput(); }); });
}

void InlineInput::refresh() {
    const App::InlineInputModel m = app_.inlineInputModel();
    if (!m.active) {
        if (!isHidden()) {
            const bool hadFocus = hasFocus();
            hide();
            if (hadFocus) viewport_->setFocus();
        }
        return;
    }
    const QString t = QString::fromStdString(m.text);
    if (isHidden()) {
        // Opened by the first digit typed in the view: it is already in the
        // text; carry on typing after it.
        setText(t);
        lastText_ = m.text;
        show();
        raise();
        setFocus();
        end(false);
    } else if (m.text != lastText_) {
        lastText_ = m.text;   // changed by App, not typed here
        if (text() != t) setText(t);
    }
    // Beside the cursor, from Qt's cursor rather than m.x/m.y: once this field
    // has the keyboard the view's input no longer tracks the mouse.
    const QPoint c = viewport_->mapFromGlobal(QCursor::pos()) + QPoint(20, -10);
    const int x = std::clamp(c.x(), 0, std::max(0, viewport_->width() - width()));
    const int y = std::clamp(c.y(), 0, std::max(0, viewport_->height() - height()));
    if (pos() != QPoint(x, y)) move(x, y);
}

// ---- dimension panel -----------------------------------------------------

DimensionPanel::DimensionPanel(App& app, QWidget* viewport) : QFrame(viewport), app_(app), viewport_(viewport) {
    setFrameShape(QFrame::StyledPanel);
    setAutoFillBackground(true);
    setFocusPolicy(Qt::NoFocus);
    setFixedWidth(240);
    App* a = &app_;

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(10, 8, 10, 10);
    auto* title = new QLabel("Dimension", this);
    QFont bold = title->font();
    bold.setBold(true);
    title->setFont(bold);
    root->addWidget(title);
    warning_ = new QLabel(this);
    warning_->setWordWrap(true);
    warning_->setStyleSheet("color: #b08000;");
    root->addWidget(warning_);
    hint_ = new QLabel(this);
    hint_->setWordWrap(true);
    root->addWidget(hint_);

    editing_ = new QWidget(this);
    auto* col = new QVBoxLayout(editing_);
    col->setContentsMargins(0, 0, 0, 0);
    kind_ = new QLabel(editing_);
    col->addWidget(kind_);
    placeHint_ = new QLabel(editing_);
    placeHint_->setWordWrap(true);
    placeHint_->setStyleSheet("color: #3a78c8;");
    col->addWidget(placeHint_);
    value_ = new ViewportField(editing_);
    col->addWidget(value_);
    driven_ = new QCheckBox("Driven (reference only)", editing_);
    driven_->setFocusPolicy(Qt::NoFocus);
    col->addWidget(driven_);
    auto* buttons = new QHBoxLayout;
    apply_ = new QPushButton(editing_);
    auto* cancel = new QPushButton("Cancel [Esc]", editing_);
    for (auto* b : {apply_, cancel}) { b->setFocusPolicy(Qt::NoFocus); buttons->addWidget(b); }
    col->addLayout(buttons);
    root->addWidget(editing_);

    connect(value_, &QLineEdit::textEdited, this, [this, a](const QString& t) {
        lastText_ = t.toUtf8().toStdString();
        const std::string s = lastText_;
        a->post([a, s] { a->setDimensionText(s); });
    });
    connect(value_, &ViewportField::submitted, this, [a] { a->post([a] { a->applyDimension(); }); });
    connect(value_, &ViewportField::cancelled, this, [a] { a->post([a] { a->cancelDimension(); }); });
    connect(apply_, &QPushButton::clicked, this, [a] { a->post([a] { a->applyDimension(); }); });
    connect(cancel, &QPushButton::clicked, this, [a] { a->post([a] { a->cancelDimension(); }); });
    connect(driven_, &QCheckBox::clicked, this, [a](bool on) { a->post([a, on] { a->setDimensionDriven(on); }); });
    hide();
}

void DimensionPanel::refresh() {
    const App::DimensionPanelModel m = app_.dimensionPanelModel();
    if (!m.open) {
        if (!isHidden()) {
            const bool hadFocus = value_->hasFocus();
            hide();
            if (hadFocus) viewport_->setFocus();
        }
        return;
    }
    warning_->setVisible(!m.warning.empty());
    warning_->setText(QString::fromStdString(m.warning));
    hint_->setVisible(!m.editing);
    hint_->setText(QString::fromStdString(m.hint));
    editing_->setVisible(m.editing);
    if (m.editing) {
        kind_->setVisible(!m.kind.empty());
        kind_->setText(QString::fromStdString(m.kind));
        placeHint_->setVisible(!m.placeHint.empty());
        placeHint_->setText(QString::fromStdString(m.placeHint));
        // App rewrites the value itself (the angle follows the mouse side
        // until typing starts); take that, but never undo what was typed.
        if (m.text != lastText_) {
            lastText_ = m.text;
            const QString t = QString::fromStdString(m.text);
            if (value_->text() != t) {
                value_->setText(t);
                if (value_->hasFocus()) value_->selectAll();
            }
        }
        if (driven_->isChecked() != m.driven) {
            const QSignalBlocker block(driven_);
            driven_->setChecked(m.driven);
        }
        apply_->setText(m.editingExisting ? "Update [Enter]" : "Apply [Enter]");
        if (m.takeFocus && !value_->hasFocus()) {
            value_->setFocus();
            value_->selectAll();
            App* a = &app_;
            app_.post([a] { a->dimensionFieldFocused(); });
        }
    } else if (value_->hasFocus()) {
        viewport_->setFocus();
    }

    const int h = sizeHint().height();
    if (height() != h) resize(width(), h);
    const QPoint at(viewport_->width() - width() - 8, 8);
    if (pos() != at) move(at);
    if (isHidden()) { show(); raise(); }
}

} // namespace shitcad
