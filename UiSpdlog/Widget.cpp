#include "Widget.h"
#include "ui_Widget.h"

#include <spdlog/sinks/base_sink.h>

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);
    ui->plainTextEdit->setReadOnly(true);
}

Widget::~Widget()
{
    delete ui;
}

void Widget::add_logging(const QString &msg, spdlog::level::level_enum level)
{
    switch (level)
    {
    case SPDLOG_LEVEL_TRACE:
    case SPDLOG_LEVEL_DEBUG:
    case SPDLOG_LEVEL_INFO:
        ui->plainTextEdit->appendHtml(msg);
        break;
    case SPDLOG_LEVEL_WARN:
        ui->plainTextEdit->appendHtml("<font color=\"blue\">" + msg + "</font>");
        break;
    case SPDLOG_LEVEL_ERROR:
        ui->plainTextEdit->appendHtml("<font color=\"gold\">" + msg + "</font>");
        break;
    case SPDLOG_LEVEL_CRITICAL:
        ui->plainTextEdit->appendHtml("<font color=\"red\">" + msg + "</font>");
        break;
    case SPDLOG_LEVEL_OFF:
        break;
    default:
        break;
    }
}

void Widget::logging_flush()
{
    // ui->textEdit->append("\r\n");
}
