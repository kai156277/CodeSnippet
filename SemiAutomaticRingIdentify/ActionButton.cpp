#include "ActionButton.h"

ActionButton::ActionButton(QAction *action, QWidget *parent)
    : QPushButton(parent)
{
    if (action)
        setAction(action);
}

void ActionButton::setAction(QAction *action)
{
    // if I've got already an action associated to the button
    // remove all connections
    if (mActionOwner && mActionOwner != action)
    {
        disconnect(mActionOwner, &QAction::changed, this, &ActionButton::updateButtonStatusFromAction);
        disconnect(this, &ActionButton::clicked, mActionOwner, &QAction::trigger);
    }

    // store the action
    mActionOwner = action;

    // configure the button
    updateButtonStatusFromAction();

    // connect the action and the button
    // so that when the action is changed the
    // button is changed too!
    connect(action, &QAction::changed, this, &ActionButton::updateButtonStatusFromAction);

    // connect the button to the slot that forwards the
    // signal to the action
    connect(this, &ActionButton::clicked, mActionOwner, &QAction::trigger);
}

void ActionButton::updateButtonStatusFromAction()
{
    if (!mActionOwner)
        return;
    setText(mActionOwner->text());
    setStatusTip(mActionOwner->statusTip());
    setToolTip(mActionOwner->toolTip());
    setIcon(mActionOwner->icon());
    setEnabled(mActionOwner->isEnabled());
    setCheckable(mActionOwner->isCheckable());
    setChecked(mActionOwner->isChecked());
}
