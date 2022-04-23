#include "WizardWidget.h"
#include "ui_WizardWidget.h"

WizardWidget::WizardWidget(QWidget *parent)
    : QWizard(parent)
    , ui(new Ui::WizardWidget)
{
    ui->setupUi(this);
}

WizardWidget::~WizardWidget()
{
    delete ui;
}

void WizardWidget::onProjectClosed()
{

    assert(false);
}

void WizardWidget::AfterInit()
{
    assert(false);
}
