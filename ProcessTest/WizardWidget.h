#pragma once

#include <QWizard>

#include <PageParam.h>

#include "WizardPage.h"

namespace Ui {
class WizardWidget;
}

class WizardWidget : public QWizard
{
    Q_OBJECT

public:
    explicit WizardWidget(QWidget *parent = nullptr);
    ~WizardWidget();

    void onProjectClosed();
    void AfterInit();

protected:
    xstype::PageParam * m_pageParams;
    QList<WizardPage *> m_pages;

    int m_curPage = 0;

    Ui::WizardWidget *ui;

private:
};
