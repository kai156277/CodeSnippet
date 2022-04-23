#include "CheckPage.h"
#include "ui_CheckPage.h"

CheckPage::CheckPage(const QString &title, QWidget *parent) :
    QWizardPage(parent),
    ui(new Ui::CheckPage)
{
    ui->setupUi(this);
}

CheckPage::~CheckPage()
{
    delete ui;
}
