#include "WizardPage.h"
#include "ui_WizardPage.h"

WizardPage::WizardPage(const QString &title, QWidget *parent)
    : QWizardPage(parent)
    , ui(new Ui::WizardPage)
{
    ui->setupUi(this);
}

WizardPage::~WizardPage()
{
    delete ui;
}

void WizardPage::OnPageParam(xstype::PageParam *param)
{
    assert(false);
}
