#pragma once

#include <QWidget>

#include <PageParam.h>
#include <ProcessThread.h>

namespace Ui {
class ProgressWidget;
}

class ProgressWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ProgressWidget(QWidget *parent = nullptr);
    ~ProgressWidget();

    void           InitThread(xstype::PageParam *param);
    QString        m_titleHint;
    ProcessThread *m_thread;

private:
    Ui::ProgressWidget *ui;
};
