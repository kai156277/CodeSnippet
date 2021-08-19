#pragma once

#include <QAction>
#include <QPushButton>

class ActionButton : public QPushButton
{
    Q_OBJECT
public:
    explicit ActionButton(QAction *action = nullptr, QWidget *parent = nullptr);

    /*!
    * Set the action owner of this button, that is the action
    * associated to the button. The button is configured immediately
    * depending on the action status and the button and the action
    * are connected together so that when the action is changed the button
    * is updated and when the button is clicked the action is triggered.
    * action the action to associate to this button
    */
    void setAction(QAction *action);

public slots:
    /*!
    * Update the button status depending on a change
    * on the action status. This slot is invoked each time the action
    * "changed" signal is emitted.
    */
    void updateButtonStatusFromAction();

private:
    QAction *mActionOwner = nullptr;
};
