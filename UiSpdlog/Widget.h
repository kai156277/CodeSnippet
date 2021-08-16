#pragma once

#include <QDebug>
#include <QWidget>

#include <spdlog/common.h>
#include <spdlog/details/log_msg.h>

QT_BEGIN_NAMESPACE
namespace Ui {
class Widget;
}
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();

public slots:
    // void add_logging(const QString &log);
    void add_logging(const QString &msg, spdlog::level::level_enum level);
    void logging_flush();

private:
    Ui::Widget *ui;
};
