#include "ProgressWidget.h"
#include "ui_ProgressWidget.h"

ProgressWidget::ProgressWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::ProgressWidget)
{
    ui->setupUi(this);
}

ProgressWidget::~ProgressWidget()
{
    delete ui;
}

void ProgressWidget::InitThread(xstype::PageParam *param)
{
}
