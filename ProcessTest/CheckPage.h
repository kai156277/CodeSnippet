#pragma once

#include <QWizardPage>

namespace Ui {
class CheckPage;
}

class CheckPage : public QWizardPage
{
    Q_OBJECT

public:
    explicit CheckPage(const QString &title, QWidget *parent = nullptr);
    ~CheckPage();

private:
    Ui::CheckPage *ui;
};
