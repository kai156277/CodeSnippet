#pragma once

#include <QWizardPage>

#include <PageParam.h>

namespace Ui {
class WizardPage;
}

class WizardPage : public QWizardPage
{
    Q_OBJECT

public:
    explicit WizardPage(const QString &title, QWidget *parent = nullptr);
    ~WizardPage();
    void OnPageParam(xstype::PageParam *param);

    QString m_titleHint;

protected:
    QString m_tempDir;

private:
    Ui::WizardPage *ui;
};
